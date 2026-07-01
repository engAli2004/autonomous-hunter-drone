# Autonomous Vision-Based Hunter-Killer Drone
**A closed-loop, multi-agent tracking system integrating ROS 2, PX4 Autopilot, and YOLOv8.**



https://github.com/user-attachments/assets/dcbaedc4-c625-4dc6-919d-b1c4c62388e7



## Executive Summary
This project bridges the gap between deep learning and aerospace control systems. It is a heterogeneous simulation where a multi-rotor drone autonomously hunts and pursues an unmanned ground vehicle (TurtleBot3) in a dynamic 3D environment. 

Instead of relying on rigid, pre-programmed waypoints, the drone uses real-time computer vision to calculate pixel-deviation error vectors, translating them into physical flight velocities via a custom Proportional Controller.

## Core Architecture & Tech Stack
* **Robotics Middleware:** ROS 2 (Humble)
* **Physics Engine:** Gazebo Classic
* **Aerospace Firmware:** PX4 Autopilot (SITL)
* **Computer Vision:** YOLOv8 Nano & OpenCV
* **Flight Communications:** MAVLink / MicroXRCE-DDS

## The Engineering Challenge
This system handles real-world robotics and aerospace integration hurdles:

* **Hardware-in-the-Loop Realism:** The drone operates using professional PX4 firmware. Flight commands respect real-world physics constraints, active GPS drift handling, and strict preflight arming failsafes.
* **Closed-Loop Visual Servoing:** The YOLOv8 pipeline calculates the center of mass of the target bounding box. A Proportional Controller maps the 2D spatial pixel error directly to 3D physical flight vectors (Roll/Pitch velocities) to dynamically adjust the intercept course.
* **Overcoming the Domain Gap:** Recognizing that AI models trained on real-world data often misclassify untextured simulated objects, the tracking pipeline bypasses raw classification labels entirely and tracks targets strictly based on dynamic bounding-box geometry.

## Installation & Setup

1. **Launch the Ground Segment (Terminal 1):**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py
2. **Launch the Target Pathing (Terminal 2):**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/multi_agent_ws/install/setup.bash 
   ros2 run agv_control random_walk 
3. **Launch the Drone Firmware (Terminal 3):**
   ```bash
   cd ~/PX4-Autopilot
   HEADLESS=1 make px4_sitl gazebo
4. **Launch the AI Vision Controller (Terminal 4):**
    ```bash
    source /opt/ros/humble/setup.bash
    cd ~/autonomous-hunter-drone
    python3 src/yolo_tracker.py

**Future Roadmap**
**Upgrade the rigid Proportional (P) Controller to a full PID Controller for smoother acceleration curves and reduced target overshoot.

Implement a depth-projection matrix to translate 2D camera pixel coordinates into a 3D spatial coordinate map.**

Upgrade the rigid Proportional (P) Controller to a full PID Controller for smoother acceleration curves and reduced target overshoot.

Implement a depth-projection matrix to translate 2D camera pixel coordinates into a 3D spatial coordinate map.
