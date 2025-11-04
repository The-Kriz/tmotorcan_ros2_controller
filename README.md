# 🦾 TMotor ROS2 Controller (AK Series)

ROS 2 (Humble) package for controlling **Cubemars AK-Series** T-Motors (e.g. AK60-6) over CAN bus using MIT-mode control.

---

## 📦 1. Project Setup

### Requirements
- ROS 2 Humble (Linux / Ubuntu 22.04)
- `python-can`, `socketcan` enabled on your system
- CAN interface configured (`can0`, `can1`, etc.)
- Permission to access CAN

## ⚙️ 2. Build Instructions

Clone inside your ROS 2 workspace:

```bash
cd ~/ROS_ws/src
git clone https://github.com/<your-username>/tmotorcan_ros2_controller.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

---

## 🧾 3. Configuration (YAML file)

Edit the file at:

```
tmotorcan_ros2_controller/config/motors.yaml
```

Example:

```yaml
tmotorcan_node:
  ros__parameters:
    num_motors: 2

    motor_1:
      name: "front_left_hip"
      type: "AK60-6"
      id: 1
      kp: 50.0
      kd: 1.0
      topic_base: "front_left_hip"

    motor_2:
      name: "front_right_hip"
      type: "AK60-6"
      id: 2
      kp: 50.0
      kd: 1.0
      topic_base: "front_right_hip"
```

📝 **Tip:**  
Each motor gets its own namespace (`motor_1`, `motor_2`, etc.) and publishes/subscribes to `/topic_base/state` and `/topic_base/command`.

---

## 🚀 4. Launch the controller

Launch file:  
`tmotorcan_ros2_controller/launch/motor.launch.py`

Run:

```bash
ros2 launch tmotorcan_ros2_controller motor.launch.py
```

You should see logs like:

```
[INFO] [tmotorcan_node]: Initialized motor 'front_left_hip' id=1 type=AK60-6 Kp=50.0 Kd=1.0 topic=/front_left_hip
[INFO] [tmotorcan_node]: Controller ready for 2 motor(s).
```

---

## 🎛️ 5. Send commands

Each motor listens on `/MOTOR_NAME/command`.

For example, send a position/velocity/torque command to **front_left_hip**:

```bash
ros2 topic pub /front_left_hip/command tmotorcan_ros2_msgs/msg/MitCommand \
"{pos: 0.0, vel: 0.0, kp: 50.0, kd: 1.0, torque: 0.2}"
```

or continuously at 10 Hz:

```bash
ros2 topic pub --rate 10 /front_left_hip/command tmotorcan_ros2_msgs/msg/MitCommand \
"{pos: 0.5, vel: 0.0, kp: 40.0, kd: 1.0, torque: 0.0}"
```

---

## 📡 6. View motor feedback

Each motor publishes feedback on `/MOTOR_NAME/state`.

To see live data:

```bash
ros2 topic echo /front_left_hip/state
```

You’ll get output like:

```text
position: 0.004
velocity: 0.000
current: 0.221
temperature: 35.0
error: 0
---
```

This updates at ~50 Hz.
