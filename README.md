Ackermann Steering Robot using ROS2 & Gazebo

This project demonstrates the design and implementation of an **Ackermann Steering Robot** in **ROS 2 Jazzy + Gazebo (Ignition)** with **vision-based navigation**.

The robot detects a target object using a camera and autonomously navigates towards it using a **custom control algorithm based on Ackermann kinematics**.

---

Project Overview

Unlike differential drive robots, Ackermann steering requires coordinated control of:

* Steering angle (front wheels)
* Forward velocity (rear wheels)

This project focuses on:

* Realistic vehicle motion (car-like steering)
* Vision-based detection and tracking
* Autonomous navigation logic using ROS2 topics

---

System Architecture

```
Camera → Vision Node → Detection Topics → Control Node → cmd_vel → Gazebo Plugin → Robot Motion
```

Components:

* **URDF/Xacro Robot Model**
* **Gazebo Simulation (Ignition Gazebo)**
* **Vision Node (OpenCV based)**
* **Control Node (State Machine Logic)**
* **Ackermann Steering Plugin**
* **ROS ↔ Gazebo Bridge**

---

Vision System

The robot uses a camera to detect a box using image processing.

📄 Source: 

Key Features:

* Two detection modes:

  * **SEARCHING (strict filters)**
  * **TRACKING (relaxed filters)**
* Filters used:

  * Contour area
  * Aspect ratio
  * Solidity
  * Corner detection
* ROI optimization:

  * Top region while searching
  * Full image while tracking

Output Topics:

* `/box_detected` → Boolean
* `/box_error` → Pixel offset
* `/box_angle` → Steering angle
* `/box_area` → Distance estimation

---

Control System

The robot follows a **state-based navigation logic**.

📄 Source: 

States:

* **INITIALIZING**
* **SCANNING (SEARCH)**
* **NAVIGATING (CURVE)**
* **CENTERING (ALIGN)**
* **DOCKED (STOP)**

Important Insight:

> Ackermann steering requires **non-zero linear velocity while turning**.
> Otherwise, the robot behaves like a differential drive (incorrect).

Control Strategy:

* Locked steering angle for stability
* Smooth curved motion instead of sharp turns
* Recovery if object is lost

---

Robot Model (URDF)

📄 Source: 

Features:

* Front steering joints
* Rear driving wheels
* Proper inertia for simulation stability
* Camera mounted on top
* Ackermann steering plugin integrated

Joints:

* Steering:

  * `FL_STEERING_JOINT`
  * `FR_STEERING_JOINT`
* Wheels:

  * `FL_WHEEL_JOINT`
  * `FR_WHEEL_JOINT`
  * `BL_WHEEL_JOINT`
  * `BR_WHEEL_JOINT`

Gazebo Integration

Ackermann Plugin:

* Converts `/cmd_vel` → realistic car motion
* Publishes:

  * `/odom`
  * `/joint_states`

Sensors:

* Camera with:

  * `/camera/image_raw`
  * `/camera/camera_info`

---

ROS ↔ Gazebo Bridge

Topics bridged include:

* `/cmd_vel`
* `/camera/image_raw`
* `/odom`
* `/joint_states`
* `/tf`

This enables communication between ROS2 nodes and Gazebo simulation.

---

How to Run

1. Build Workspace

```bash
cd ~/ros2_ws3
colcon build
source install/setup.bash
```

2. Launch Simulation

```bash
ros2 launch ackerman_bot sim.launch.py
```

3. Run Vision Node

```bash
ros2 run ackerman_bot vision_node
```

4. Run Control Node

```bash
ros2 run ackerman_bot control_node
```

Manual Control (Optional)

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 1.0
angular:
  z: 0.3
"
```

---

Key Learnings

* Difference between **Ackermann vs Differential Drive**
* Importance of **linear velocity in steering**
* Handling real-world vision noise in simulation
* ROS2 topic communication and debugging
* Gazebo plugin integration and bridging

---

Challenges Faced

* Wheels rotating in opposite directions
* Missing inertial values in URDF
* Plugin loading errors
* ROS-Gazebo topic mismatch
* Object detection instability near camera

---

Solutions Implemented

* Correct joint configuration
* Added proper inertial parameters
* Used correct Gazebo system plugins
* Implemented dual-mode vision filtering
* Stabilized control logic with state machine

---

Future Improvements

* SLAM integration
* Navigation (Nav2)
* Obstacle avoidance
* Real-world hardware implementation


Author

**Manjunadh Kotturi**
Robotics & Embedded Systems Enthusiast
