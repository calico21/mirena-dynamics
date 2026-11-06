# ROS 2 Interface Contract — TeR26 (`MirenaSim`)

## 1. Subscribed Topics

| Topic | Message Type | QoS | Fields & Units | Notes |
| :--- | :--- | :--- | :--- | :--- |
| `/control` | `mirena_common/msg/CarControl` | Default (`RELIABLE`, depth 10) | `float32 gas` $\in [-1, 1]$ (`>0` throttle, `<0` brake)<br>`float32 steer_angle` (rad, `+` = left/CCW) | Consumed when `pilot == PilotMode.ROS`. |
| `/system/mission_status` | `mirena_common/msg/MissionInfo` | Default | `string mission`, `uint8 status` | Used by AS state machine. |
| `/as_status` | `mirena_common/msg/AsInfo` | Default | `uint8 status` (`AS_OFF`, `AS_READY`, `AS_DRIVING`, `AS_FINISHED`, `AS_EMERGENCY`) | Enforces EBS brake lock when in `AS_EMERGENCY` or `AS_FINISHED`. |

## 2. Published Topics

| Topic | Message Type | Rate & QoS | Frame ID | Notes |
| :--- | :--- | :--- | :--- | :--- |
| `/sensors/wheel_speeds` | `mirena_common/msg/WheelSpeeds` | 100 Hz (`BEST_EFFORT`, `VOLATILE`, depth 5) | `"car/cog"` | `fl`, `fr`, `rl`, `rr` in **RPM** ($\omega_{w,i} \cdot \frac{60}{2\pi}$). |
| `/debug/state/car` | `mirena_common/msg/Car` | 400 Hz (Default) | `"odom"` $\to$ `"car/cog"` | Full ground-truth pose, twist, acceleration, slip angles, wheel forces. |
| `/sensors/imu` | `sensor_msgs/msg/Imu` | 200 Hz (`RosIns`) | `"car/imu"` | Specific force ($\mathbf{a} - R^T \mathbf{g}$), angular velocity, orientation. |
| `/sensors/imu/odom` | `nav_msgs/msg/Odometry` | 200 Hz (`RosIns`) | `"odom"` $\to$ `"car/imu"` | Integrated INS pose and twist. |
| `/sensors/gps/fix` | `sensor_msgs/msg/NavSatFix` | 20 Hz (`RosIns`) | `"car/imu"` | Simulated GNSS WGS84 coordinates. |
| `/sensors/lidar/points` | `sensor_msgs/msg/PointCloud2` | 10–20 Hz (`RosLidar`) | `"car/lidar"` | 3D LiDAR point cloud. |
| `/sensors/camera/image` | `sensor_msgs/msg/Image` | 30 Hz (`RosCamera`) | `"car/camera"` | RGB camera stream. |
| `/inferred_control` | `mirena_common/msg/CarControl` | 400 Hz | N/A | Active commands (`gas`, `steer_angle`). |
| `/lap_status` | `mirena_common/msg/LapStatus` | Event / Tick | N/A | Lap counter and timing. |
