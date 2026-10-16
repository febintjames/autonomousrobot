# Real-robot navigation guide (ESP32 + RPLidar + Nav2 + web app)

This guide explains why the robot was moving badly, what was changed, and how to
run everything, including sending the robot to **table1** from a phone.

---

## 1. What was wrong

| # | Problem found in the repo | What you see on the robot | Fix |
|---|---|---|---|
| 1 | `nav2.launch.py` always loaded `config/bcr_map.yaml`. That is the **Gazebo warehouse map** (29 x 31 m with shelves), not your room. | AMCL tries to match your real walls to warehouse walls. The pose jumps, paths go through real walls, and the robot turns the wrong way or gets lost. | `nav2.launch.py` now takes `map:=...`. Make a map of your room (section 4). |
| 2 | `esp32_driver` **and** `rf2o` both publish `/odom` **and** the `odom -> base_link` TF (rf2o's topic was changed from `/odom_rf2o` to `/odom`). | If both run, the robot pose flips between two estimates many times a second, so the robot jerks, oscillates or spins. | New `robot_bringup.launch.py` starts exactly one odometry source (`odom_source:=wheel` or `laser`). |
| 3 | `cmd_vel_serial` sends only `F/B/L/R` commands. | It can't drive forward and turn at the same time, it ignores turns slower than 0.2 rad/s, and it gives no odometry. Nav2 needs smooth curves. | Use `esp32_driver` (`s <vl> <vr>` wheel speeds) with Nav2. Keep `cmd_vel_serial` only for simple tests. |
| 4 | Nav2 allowed 0.26 m/s and 1.0 rad/s, but `esp32_driver` silently cut speeds to 0.18 m/s and 0.5 rad/s, clamping each one **separately**. | Changing `v` and `w` separately changes the turn radius, so the robot drives a different curve than the one Nav2 planned and keeps drifting off the path. | Nav2 limits now match the driver (0.18 / 0.5). The driver now scales `v` and `w` **together**, so the curve stays the same. |
| 5 | `local_costmap robot_radius: 0.12`, `global_costmap robot_radius: 0.22`. The robot is about 38 cm wide. | The controller thinks the robot is tiny, so it cuts corners and brushes tables and chair legs. | Both are now `0.22`. **Measure your robot** and adjust (half the largest diagonal + 2 cm). |
| 6 | Acceleration `2.5 m/s²` (a simulation value). Progress checker `0.5 m in 10 s`. | Jerky starts and wheel slip, which ruins odometry. Goals were aborted with "Failed to make progress" while the robot rotated in place. | Accel 0.4 m/s² (decel 0.6), progress checker 0.25 m in 15 s. |
| 7 | AMCL `laser_max_range: 100`, `laser_min_range: -1`. | Hits on the robot's own body are used for localization. | 12 m / 0.20 m (same as slam_toolbox). |
| 8 | No `base_link -> laser` transform anywhere in the repo. | Without it, scans can't be used. If it's set by hand with the wrong yaw, obstacles show up behind the robot. | `robot_bringup.launch.py` publishes it with `laser_x/y/z/yaw` arguments. |
| 9 | `remapper.py` (cmd_vel -> `bcr_bot/cmd_vel`) ran in `nav2.launch.py`. | Nothing; it only matters in Gazebo. | Removed from the real-robot launch. |

---

## 2. One-time setup on the robot computer

```bash
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup \
                 ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-teleop-twist-keyboard python3-serial
sudo usermod -aG dialout $USER     # log out/in once afterwards

cd ~/ros2_ws/src && git clone https://github.com/adithyaps-sudo/nav2.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

**USB ports swap after reboots** (`ttyUSB0` and `ttyUSB1`). Use stable names instead:

```bash
ls -l /dev/serial/by-id/
# then e.g. lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_...  esp32_port:=/dev/serial/by-id/usb-1a86_...
```

---

## 3. Check the hardware BEFORE using Nav2 (do not skip)

Start the robot (lidar, TF and ESP32):

```bash
ros2 launch bcr_bot robot_bringup.launch.py lidar_port:=/dev/ttyUSB0 esp32_port:=/dev/ttyUSB1
```

**a. Only one odometry publisher**

```bash
ros2 topic info /odom          # "Publisher count: 1"
ros2 run tf2_ros tf2_echo odom base_link
```

**b. Lidar direction.** Open RViz with fixed frame `base_link` and add LaserScan `/scan`.
Hold a box 50 cm **in front** of the robot. It must appear on the **red (X) axis side**.
If it appears behind the robot, relaunch with `laser_yaw:=3.14159`. Also set
`laser_x`/`laser_y` if the lidar is not above the midpoint between the drive wheels.

**c. Driving direction** (in another terminal):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic echo /odom --field pose.pose
```

- Press `i` (forward): the robot must move forward and `/odom` x must **increase**.
- Press `j` (turn left): the robot must turn **counter-clockwise** and yaw must **increase**.
- If odom goes the wrong way, set `invert_left_encoder` / `invert_right_encoder`.
  If the robot itself turns the wrong way, swap the motor wires or the left/right order in the ESP32 firmware.

**d. Calibrate odometry.** This is what makes the robot drive straight to a table.

```bash
ros2 run my_python_pkg odom_check     # prints "distance X m | rotation Y deg" since start
```

1. Mark the floor and drive exactly **1.00 m** straight.
   Set `linear_scale = 1.00 / distance` (for example, it shows 0.930 m, so `linear_scale: 1.075`).
   Restart the driver with the new value before step 2, because `linear_scale` also affects rotation.
2. Restart `odom_check`, then spin in place exactly **5 turns** (watch a tape mark on the floor).
   Set `wheel_separation = 0.34 * rotation / 1800` (for example, 1910 deg gives 0.361).
   Real tracks are usually a bit wider than measured because the tyres scrub.

Put both values in `bcr_bot/launch/robot_bringup.launch.py` (esp32 `parameters`).

**e. Slow-speed test.** Near goals and in turns, Nav2 slows down to **0.05 m/s**:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

If the wheels don't turn, the ESP32 is driving the motors with plain PWM and 0.05 m/s
is below the motors' dead zone. Either add a speed PID / minimum PWM in the firmware
(best), or raise `regulated_linear_scaling_min_speed` and `min_approach_linear_velocity`
in `nav2_params.yaml` to the slowest speed that really moves the robot.

---

## 4. Make a map of YOUR room

```bash
# terminal 1
ros2 launch bcr_bot robot_bringup.launch.py
# terminal 2
ros2 launch bcr_bot mapping.launch.py
# terminal 3: drive slowly around the whole room and return to the start
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# when the map looks complete in RViz:
mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

Tips: drive slowly, turn slowly, and close a loop by coming back to the start. Doors
and chairs that move between runs will show up as obstacles; tables are fine.

---

## 5. Navigate

```bash
# terminal 1: robot
ros2 launch bcr_bot robot_bringup.launch.py
# terminal 2: Nav2 with YOUR map (default is ~/maps/my_map.yaml)
ros2 launch bcr_bot nav2.launch.py map:=$HOME/maps/my_map.yaml
#   headless on the robot computer:  use_rviz:=False
# terminal 3: web app
ros2 launch table_nav_web table_nav_web.launch.py
```

Before every run, tell AMCL where the robot is. Use **2D Pose Estimate** in RViz, or the web
app (**Robot is here**). Until then Nav2 has no `map -> odom` transform and won't move.

---

## 6. The web app: "go to table1"

The terminal prints the address, e.g. `Web app running: http://192.168.1.50:8080/`.
Open it on any phone or laptop on the **same Wi-Fi**.

**Save the tables once:**

1. Set the robot's start position (**Robot is here** on the map, or RViz 2D Pose Estimate).
2. Drive the robot (teleop) to table 1, stopping where it should wait and facing the way it should face.
3. Type `table1` and press **Save current position**. Repeat for `table2`, `home`, `kitchen`, ...

You can also tap a spot on the map and drag to set the heading, then choose **Save as place...**.

**Daily use:** tap **table1**. The page shows "Going to table1, 2.1 m left, about 12 s", then
"Arrived at table1". **STOP** cancels any Nav2 goal, including ones sent from RViz.

Places are stored in `~/.ros/table_locations.yaml` (x, y in metres, yaw in radians, map frame).
If you make a new map, save the places again, because the coordinates belong to one map.

**From another program** (a POS system, a button box, a script):

```bash
curl -X POST http://192.168.1.50:8080/api/goto   -H 'Content-Type: application/json' -d '{"name": "table1"}'
curl -X POST http://192.168.1.50:8080/api/cancel -H 'Content-Type: application/json' -d '{}'
curl http://192.168.1.50:8080/api/status
```

The page has **no login**, so anyone on the same Wi-Fi can drive the robot.
Keep it on a private network.

---

## 7. Troubleshooting

| Symptom | Most likely cause | What to do |
|---|---|---|
| Robot snakes or oscillates along the path | Odometry not calibrated, or two odom publishers | Section 3a and 3d |
| Robot pose jumps in RViz | Wrong map, wrong initial pose, or lidar yaw wrong | Sections 4, 5 and 3b |
| Robot spins in place and never drives | Encoder or motor direction wrong (3c), or lidar mounted backwards (3b) | Fix direction first |
| "Failed to make progress", goal aborted | Motors stall at low speed | Section 3e |
| Stops 20-25 cm short of the table | `xy_goal_tolerance: 0.25` | After calibration, lower it to `0.10` in `nav2_params.yaml` |
| Won't pass between tables | Gap < 2 x `robot_radius`, or inflation too big | Measure the robot; try `inflation_radius: 0.40` |
| RViz goals work, web page says "Nav2 is not running" | Web app started in a different `ROS_DOMAIN_ID` or terminal without `source` | `source install/setup.bash` in every terminal |
| Nav2 runs but motors never move | Newer Nav2 (Kilted) publishes `TwistStamped` on `/cmd_vel` | `ros2 topic info /cmd_vel -v`; set `enable_stamped_cmd_vel: false` for the Nav2 servers or make the driver accept `TwistStamped` |
| Phone can't open the page | Different network or firewall | Use the IP from the log; `sudo ufw allow 8080` |

**Optional upgrade later:** fuse wheel odometry + rf2o (+ an IMU if you add one) with
`robot_localization` EKF. Send rf2o to `/odom_rf2o` and the wheels to `/odom_wheel`, both with
`publish_tf: false`, and let the EKF publish `/odom` and the TF.
