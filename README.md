# Autonomous Vision-Based Hunter-Killer Drone
**A closed-loop, multi-agent tracking system integrating ROS 2, PX4 Autopilot, and YOLOv8.**

<video src="media/chase_demo.mp4" autoplay loop muted playsinline width="100%"></video>

## Executive Summary
This project bridges the gap between deep learning and aerospace control systems. It is a heterogeneous simulation where a multi-rotor drone autonomously hunts and pursues an unmanned ground vehicle (TurtleBot3) in a dynamic 3D environment. 

Instead of relying on rigid, pre-programmed waypoints, the drone uses real-time computer vision to calculate pixel-deviation error vectors, translating them into physical flight velocities via a Proportional Controller.

## Core Architecture & Tech Stack
* **Robotics Middleware:** ROS 2 (Humble)
* **Physics Engine:** Gazebo Classic
* **Aerospace Firmware:** PX4 Autopilot (SITL)
* **Computer Vision:** YOLOv8 Nano & OpenCV
* **Flight Communications:** MAVLink

## The Engineering Challenge (Unique Value)
This system was built to handle real-world robotics and aerospace integration challenges:

* **Hardware-in-the-Loop Realism:** The drone operates using professional PX4 firmware. Flight commands must respect real-world physics, active GPS drift handling, and strict preflight battery failsafes.
* **Closed-Loop Visual Servoing:** The YOLOv8 pipeline calculates the center of mass of the target bounding box. A custom Proportional Controller maps the 2D spatial pixel error directly to 3D physical flight vectors (Roll/Pitch velocities).
* **Overcoming the Domain Gap:** Recognizing that AI models trained on real-world data often misclassify untextured simulated objects (e.g., labeling a ground robot as an "airplane"), the pipeline ignores classification labels entirely and tracks purely based on dynamic bounding-box geometry.

## Installation & Setup

1. **Launch the Ground Segment (Terminal 1):**
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py
