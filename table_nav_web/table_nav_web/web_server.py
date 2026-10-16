#!/usr/bin/env python3
"""
Table navigation web app: open http://<robot-ip>:8080 on a phone and send the
robot to saved places such as "table1".

  * Saved places live in a YAML file (default ~/.ros/table_locations.yaml).
  * "Go" sends a Nav2 NavigateToPose goal and shows live progress.
  * "Stop" cancels the goal (Nav2 then publishes zero velocity).
  * "Save current position" stores the robot's pose (map -> base_link TF).
  * "Robot is here" publishes /initialpose so AMCL knows where the robot starts.

Threading: rclpy spins on the main thread and owns every ROS call. The HTTP
server runs in its own threads; they only read cached state (under a lock)
and put commands on a queue that a timer drains on the ROS thread.
"""
import base64
import math
import os
import queue
import socket
import threading

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from table_nav_web.http_api import LocationStore, make_server


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def duration_to_sec(d):
    return d.sec + d.nanosec / 1e9


class TableNavWeb(Node):

    def __init__(self):
        super().__init__('table_nav_web')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('locations_file', '~/.ros/table_locations.yaml')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('action_name', 'navigate_to_pose')

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.locations_file = self.get_parameter('locations_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        action_name = self.get_parameter('action_name').value

        self._lock = threading.Lock()
        self._commands = queue.Queue()
        self._pose = None
        self._pose_failures = 0
        self._map = None
        self._map_version = 0
        self._nav2_ready = False
        self._nav = self._nav_state('idle')
        self._goal_handle = None
        self._goal_seq = 0
        self._cancel_pending = False

        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        # An empty CancelGoal request (zero id, zero stamp) cancels ALL goals,
        # so Stop also works for goals sent from RViz or another program.
        self._cancel_all_client = self.create_client(
            CancelGoal, f'{action_name}/_action/cancel_goal')
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(0.1, self._process_commands)
        self.create_timer(0.2, self._update_pose)
        self.create_timer(1.0, self._check_nav2)

    # ------------------------------------------------------------------
    # Backend API used by the HTTP threads (no ROS calls in here)
    # ------------------------------------------------------------------
    def get_status(self):
        with self._lock:
            return {
                'pose': dict(self._pose) if self._pose else None,
                'nav': dict(self._nav),
                'nav2_ready': self._nav2_ready,
                'map_version': self._map_version if self._map else None,
            }

    def get_pose(self):
        with self._lock:
            return dict(self._pose) if self._pose else None

    def get_map(self):
        with self._lock:
            return self._map

    def go_to(self, name, x, y, yaw):
        with self._lock:
            ready = self._nav2_ready
        if not ready:
            return False, 'Nav2 is not running (no navigate_to_pose action server found)'
        self._commands.put(('goto', name, x, y, yaw))
        return True, f'Sending robot to {name}'

    def cancel(self):
        self._commands.put(('cancel',))
        return True, 'Stopping robot'

    def set_initial_pose(self, x, y, yaw):
        self._commands.put(('initial_pose', x, y, yaw))
        return True, 'Initial pose sent to AMCL'

    # ------------------------------------------------------------------
    # ROS side (runs on the executor thread)
    # ------------------------------------------------------------------
    @staticmethod
    def _nav_state(state, target=None, message=''):
        return {'state': state, 'target': target, 'message': message,
                'distance_remaining': None, 'eta': None, 'recoveries': 0}

    def _set_nav(self, **fields):
        with self._lock:
            self._nav.update(fields)

    def _process_commands(self):
        while True:
            try:
                cmd = self._commands.get_nowait()
            except queue.Empty:
                return
            if cmd[0] == 'goto':
                self._send_goal(*cmd[1:])
            elif cmd[0] == 'cancel':
                self._cancel_goal()
            elif cmd[0] == 'initial_pose':
                self._publish_initial_pose(*cmd[1:])

    def _send_goal(self, name, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        q = goal.pose.pose.orientation
        q.x, q.y, q.z, q.w = yaw_to_quaternion(float(yaw))

        # A new goal on the same action server pre-empts the current one in
        # Nav2, so results from older goals are ignored using this sequence id.
        self._goal_seq += 1
        seq = self._goal_seq
        self._cancel_pending = False
        with self._lock:
            self._nav = self._nav_state('sending', name)
        self.get_logger().info(f'Goal -> {name}: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=lambda msg: self._on_feedback(seq, msg))
        future.add_done_callback(lambda f: self._on_goal_response(seq, f))

    def _on_goal_response(self, seq, future):
        if seq != self._goal_seq:
            return
        try:
            goal_handle = future.result()
        except Exception as e:  # noqa: BLE001 - report any transport error to the UI
            self._set_nav(state='failed', message=f'Could not send goal: {e}')
            return
        if not goal_handle.accepted:
            self._set_nav(state='failed', message='Nav2 rejected the goal (is Nav2 active?)')
            return
        self._goal_handle = goal_handle
        self._set_nav(state='navigating')
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._on_result(seq, f))
        if self._cancel_pending:
            self._cancel_pending = False
            goal_handle.cancel_goal_async()

    def _on_feedback(self, seq, msg):
        if seq != self._goal_seq:
            return
        fb = msg.feedback
        self._set_nav(
            distance_remaining=round(float(fb.distance_remaining), 2),
            eta=round(duration_to_sec(fb.estimated_time_remaining), 1),
            recoveries=int(fb.number_of_recoveries),
        )

    def _on_result(self, seq, future):
        if seq != self._goal_seq:
            return
        self._goal_handle = None
        response = future.result()
        status = response.status
        with self._lock:
            target = self._nav['target']
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._set_nav(state='arrived', message=f'Arrived at {target}', distance_remaining=0.0)
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_nav(state='canceled', message='Stopped')
        else:
            # Jazzy+ results carry an error code; Kilted+ also a message.
            result = response.result
            code = getattr(result, 'error_code', 0)
            text = getattr(result, 'error_msg', '')
            detail = text or (f'error code {code}' if code else 'see the Nav2 terminal')
            self._set_nav(state='failed', message=f'Navigation failed: {detail}')
        self.get_logger().info(f'Goal {target} finished with status {status}')

    def _cancel_goal(self):
        with self._lock:
            state = self._nav['state']
        if state == 'sending':
            self._cancel_pending = True  # also cancel it as soon as Nav2 accepts it
        if self._cancel_all_client.service_is_ready():
            self._cancel_all_client.call_async(CancelGoal.Request())
            self._set_nav(message='Stopping...')
        elif self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._set_nav(message='Stopping...')

    def _publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        q = msg.pose.pose.orientation
        q.x, q.y, q.z, q.w = yaw_to_quaternion(float(yaw))
        msg.pose.covariance[0] = 0.25    # x variance (m^2)
        msg.pose.covariance[7] = 0.25    # y variance
        msg.pose.covariance[35] = 0.0685  # yaw variance (~15 deg)
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(f'Initial pose: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')

    def _update_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            self._pose_failures += 1
            if self._pose_failures > 10:  # ~2 s without map -> base_link
                with self._lock:
                    self._pose = None
            return
        self._pose_failures = 0
        tr = t.transform.translation
        with self._lock:
            self._pose = {'x': round(tr.x, 3), 'y': round(tr.y, 3),
                          'yaw': round(quaternion_to_yaw(t.transform.rotation), 3)}

    def _check_nav2(self):
        ready = self._nav_client.server_is_ready()
        with self._lock:
            self._nav2_ready = ready

    def _on_map(self, msg):
        # Occupancy 0..100 stays as is, unknown (-1) becomes 255.
        cells = np.asarray(msg.data, dtype=np.int8).astype(np.uint8)
        info = msg.info
        grid = {
            'frame_id': msg.header.frame_id,
            'width': info.width,
            'height': info.height,
            'resolution': info.resolution,
            'origin': {'x': info.origin.position.x, 'y': info.origin.position.y,
                       'yaw': quaternion_to_yaw(info.origin.orientation)},
            'data': base64.b64encode(cells.tobytes()).decode('ascii'),
        }
        with self._lock:
            self._map = grid
            self._map_version += 1
        self.get_logger().info(f'Map received: {info.width}x{info.height} @ {info.resolution} m')


def _lan_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))  # no packet is sent for UDP connect
            return s.getsockname()[0]
    except OSError:
        return 'localhost'


def main(args=None):
    rclpy.init(args=args)
    node = TableNavWeb()

    share = get_package_share_directory('table_nav_web')
    store = LocationStore(node.locations_file,
                          seed_path=os.path.join(share, 'config', 'locations.yaml'))
    server = make_server(node, store, os.path.join(share, 'web'), node.host, node.port)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    node.get_logger().info(f'Web app running: http://{_lan_ip()}:{node.port}/')
    node.get_logger().info(f'Saved places file: {store.path}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
