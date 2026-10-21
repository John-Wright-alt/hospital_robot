# Hospital Guide Robot ("Anna") — Final-year project, Stellenbosch University

Mobile robot that welcomes a patient, asks for a destination on a touchscreen,
guides them there with ROS 2 Nav2, offers to wait, and returns.

Ubuntu 24.04, ROS 2 Jazzy, Raspberry Pi 4, Raspberry Pi Pico W, ESP32-S3, LD06 LiDAR.

| Folder | Contents |
|---|---|
| `hospital_robot_description/` | URDF / robot model |
| `hospital_robot_base/` | Pico bridge, odometry, LiDAR/IMU, head column, ultrasonics |
| `hospital_robot_nav/` | SLAM, Nav2 configuration, mission node, waypoints |
| `hospital_robot_hri/` | Eyes, voice, touchscreen interaction ("Anna") |
| `firmware/pico_motor_firmware/` | Pico W: FOC motor control, lift, ultrasonics, safety |
| `firmware/HeadColumn/` | ESP32-S3: face detection for the height-adjustable screen |
| `maps/` | Maps and waypoints used in testing |
| `docs/` | Build and test guides for each phase |

Third-party: LD06 driver from https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
(clone it into the workspace `src/` folder).

Build: `cd ~/robot_ws && colcon build --symlink-install`
