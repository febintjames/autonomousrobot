#!/usr/bin/env python3
"""
=============================================================================
ESP32 Teleop Twist Subscriber & Odometry Publisher for Nav2 / SLAM
=============================================================================
This standalone ROS 2 node connects your robot hardware (ESP32 + MDD10A + Encoders)
to the ROS 2 navigation stack (SLAM Toolbox / Nav2).

How it works:
  1. Subscribes to `/cmd_vel` (geometry_msgs/Twist)
     - Driven by `teleop_twist_keyboard` during manual driving / SLAM
     - Driven automatically by `Nav2` during autonomous navigation
  2. Translates (vx, wz) into left & right wheel speeds and sends "s <vl> <vr>\n" to ESP32
  3. Receives live encoder tick counts from ESP32 ("e <left_ticks> <right_ticks>\n") at ~30 Hz
  4. Computes robot odometry (pose x, y, yaw and speeds vx, vtheta)
  5. Publishes `/odom` (nav_msgs/Odometry) and TF transform (odom -> base_link or base_footprint)
=============================================================================
"""

import math
import sys
import threading
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class ESP32TeleopSubscriberNode(Node):

    def __init__(self):
        super().__init__('esp32_teleop_odometry_node')

        # -------------------------------------------------------------
        # PARAMETERS
        # -------------------------------------------------------------
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.0425)         # 0.0425 m = 85mm diameter wheel
        self.declare_parameter('wheel_separation', 0.34)       # 0.34 m = Center-to-center track width (38cm outer)
        self.declare_parameter('ticks_per_revolution', 537.6)  # 7 PPR * 19.2 gear ratio * 4x CPR
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')      # matches nav2_params.yaml base_link
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('invert_left_encoder', False)
        self.declare_parameter('invert_right_encoder', False)
        self.declare_parameter('max_linear_speed', 0.18)       # 0.18 m/s (~0.65 km/h) - perfect gentle mapping pace
        self.declare_parameter('max_angular_speed', 0.50)      # 0.50 rad/s (~28 deg/s) - gives Nav2 headroom to turn

        # Fetch parameter values
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        self.invert_left = self.get_parameter('invert_left_encoder').get_parameter_value().bool_value
        self.invert_right = self.get_parameter('invert_right_encoder').get_parameter_value().bool_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        self.get_logger().info('====================================================')
        self.get_logger().info('ESP32 Teleop Subscriber & Odometry Publisher Initializing')
        self.get_logger().info(f'Port: {self.port} | Baud: {self.baud_rate}')
        self.get_logger().info(f'Wheel Radius: {self.wheel_radius}m | Track: {self.wheel_separation}m')
        self.get_logger().info(f'Ticks/Rev: {self.ticks_per_rev} (7 PPR x 19.2 x 4)')
        self.get_logger().info(f'Listening on: {cmd_vel_topic} | Publishing to: {odom_topic}')
        self.get_logger().info(f'TF Base Frame: {self.base_frame} | Odom Frame: {self.odom_frame}')
        self.get_logger().info('====================================================')

        # Serial Connection State
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.is_connected = False
        self._connect_serial()

        # Odometry State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0

        self.prev_ticks_left = None
        self.prev_ticks_right = None
        self.prev_time = self.get_clock().now()

        # Command tracking
        self.target_v_left = 0.0
        self.target_v_right = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Publishers & Broadcasters
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to /cmd_vel
        self.cmd_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_cb, 10)

        # Background thread for serial reading
        self.running = True
        self.serial_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.serial_thread.start()

        # Timer to stream commands to ESP32 at 20 Hz
        self.cmd_timer = self.create_timer(0.05, self._send_cmd_to_esp32)

    def _connect_serial(self):
        try:
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                self.serial_conn = serial.Serial(
                    port=self.port,
                    baudrate=self.baud_rate,
                    timeout=0.5,
                    write_timeout=0.5,
                    dsrdtr=False,
                    rtscts=False
                )
                self.serial_conn.dtr = False
                self.serial_conn.rts = False
                time.sleep(0.3)
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                self.is_connected = True
                self.get_logger().info(f'Connected to ESP32 on {self.port}')
        except Exception as e:
            self.is_connected = False
            self.get_logger().warn(f'Unable to open port {self.port}: {e}. Retrying...')

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Limit speed by scaling v and w together. Clamping them separately changes
        # the turn radius (w / v), so the robot would drive a different arc than
        # the one Nav2 planned and keep drifting off the path.
        scale = 1.0
        if abs(v) > self.max_linear_speed:
            scale = min(scale, self.max_linear_speed / abs(v))
        if abs(w) > self.max_angular_speed:
            scale = min(scale, self.max_angular_speed / abs(w))
        v *= scale
        w *= scale

        # Differential kinematics:
        # v_l = v - (w * W / 2)
        # v_r = v + (w * W / 2)
        half_track = (w * self.wheel_separation) / 2.0
        self.target_v_left = v - half_track
        self.target_v_right = v + half_track
        self.last_cmd_time = self.get_clock().now()

    def _send_cmd_to_esp32(self):
        if not self.is_connected or not self.serial_conn or not self.serial_conn.is_open:
            return

        # Watchdog timeout: if /cmd_vel stops arriving (> 0.5s), send 0.0
        now = self.get_clock().now()
        dt_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt_cmd > 0.5:
            vl = 0.0
            vr = 0.0
        else:
            vl = self.target_v_left
            vr = self.target_v_right

        cmd = f"s {vl:.3f} {vr:.3f}\n"
        try:
            with self.serial_lock:
                self.serial_conn.write(cmd.encode('ascii'))
        except Exception as e:
            self.get_logger().warn(f'Serial write failed: {e}')
            self.is_connected = False

    def _read_serial_loop(self):
        while self.running and rclpy.ok():
            if not self.is_connected:
                time.sleep(2.0)
                self._connect_serial()
                continue

            try:
                if self.serial_conn and self.serial_conn.is_open:
                    line = self.serial_conn.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        self._parse_line(line)
                else:
                    self.is_connected = False
            except serial.SerialException as e:
                if not self.running or not rclpy.ok():
                    break
                err_str = str(e)
                if 'returned no data' in err_str or 'EAGAIN' in err_str or 'Resource temporarily unavailable' in err_str:
                    time.sleep(0.05)
                    continue
                self.get_logger().warn(f'Serial read error: {e}')
                self.is_connected = False
                time.sleep(1.0)
            except Exception as e:
                if not self.running or not rclpy.ok():
                    break
                self.get_logger().warn(f'Serial read error: {e}')
                self.is_connected = False
                time.sleep(1.0)

    def _parse_line(self, line: str):
        if line.startswith('e ') or line.startswith('E '):
            parts = line.split()
            if len(parts) >= 3:
                try:
                    left_ticks = int(parts[1])
                    right_ticks = int(parts[2])
                    self._update_odometry(left_ticks, right_ticks)
                except ValueError:
                    pass
        elif 'ESP32' in line or 'OK' in line:
            self.get_logger().info(f'[ESP32] {line}')
        else:
            parts = line.split()
            if len(parts) == 2:
                try:
                    left_ticks = int(parts[0])
                    right_ticks = int(parts[1])
                    self._update_odometry(left_ticks, right_ticks)
                except ValueError:
                    pass

    def _update_odometry(self, left_ticks: int, right_ticks: int):
        now = self.get_clock().now()

        if self.prev_ticks_left is None or self.prev_ticks_right is None:
            self.prev_ticks_left = left_ticks
            self.prev_ticks_right = right_ticks
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0001:
            return

        delta_l = left_ticks - self.prev_ticks_left
        delta_r = right_ticks - self.prev_ticks_right
        if self.invert_left:
            delta_l = -delta_l
        if self.invert_right:
            delta_r = -delta_r

        self.prev_ticks_left = left_ticks
        self.prev_ticks_right = right_ticks
        self.prev_time = now

        # Convert encoder ticks to distance (meters)
        m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = delta_l * m_per_tick * self.linear_scale
        d_right = delta_r * m_per_tick * self.linear_scale

        d_center = (d_right + d_left) / 2.0
        d_theta = ((d_right - d_left) / self.wheel_separation) * self.angular_scale

        # Integrate pose
        if abs(d_theta) < 1e-6:
            self.x += d_center * math.cos(self.th)
            self.y += d_center * math.sin(self.th)
        else:
            self.x += d_center * math.cos(self.th + (d_theta / 2.0))
            self.y += d_center * math.sin(self.th + (d_theta / 2.0))

        self.th += d_theta
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        self.vx = d_center / dt
        self.vth = d_theta / dt

        # Publish Odometry & TF
        q = quaternion_from_euler(0.0, 0.0, self.th)

        # 1. TF Transform: odom -> base_link
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf_msg)

        # 2. Odometry Message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.pose.covariance = [
            1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-2
        ]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth

        odom.twist.covariance = [
            1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-2
        ]

        self.odom_pub.publish(odom)

    def destroy_node(self):
        self.running = False
        try:
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.write(b"s 0.0 0.0\n")
                    self.serial_conn.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32TeleopSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
