# 🚗 Ackermann Steering Model for Odometry and Localization

This project implements the **Ackermann steering kinematic model** and its **bicycle approximation**, designed for estimating the pose of an autonomous car navigating the Monza F1 circuit. The project was developed using the **ROS 1** environment, with input from encoder and GPS sensors, and includes an **Extended Kalman Filter (EKF)** to fuse sensor data and improve localization accuracy.

## 📘 Table of Contents

- [Overview](#overview)
- [Theoretical Background](#theoretical-background)
  - [Ackermann Steering Geometry](#ackermann-steering-geometry)
  - [Bicycle Model Approximation](#bicycle-model-approximation)
- [Mathematical Models](#mathematical-models)
  - [Non-Holonomic Constraints](#non-holonomic-constraints)
  - [Kinematic Bicycle Model Equations](#kinematic-bicycle-model-equations)
- [ROS Implementation](#ros-implementation)
- [Sensor Fusion with EKF](#sensor-fusion-with-ekf)
- [Output](#output)
- [Dependencies](#dependencies)

---

## 🔍 Overview

The goal is to estimate the real-time position (x, y, θ) of an autonomous car using a simplified vehicle model and limited sensor data:

- **Inputs**:
  - Wheel encoder data (linear speed and steering angle)
  - Dual GPS data (front and rear GPS units)
  
- **Outputs**:
  - Odometry estimation in the world frame
  - GPS-based position transformation to Cartesian frame
  - EKF-based fusion of odometry and GPS for improved accuracy

---

## 📐 Theoretical Background

### Ackermann Steering Geometry

Ackermann steering allows the vehicle to turn without lateral slipping of the tires. The turning radii of the inner and outer wheels differ due to the fixed geometry of the front axle:

**Ackermann Condition:**

\[
\tan(\delta_R) = \frac{L}{R + \frac{W}{2}}, \quad \tan(\delta_L) = \frac{L}{R - \frac{W}{2}}
\]

Where:
- \( \delta_R, \delta_L \) = right and left front wheel steering angles  
- \( L \) = wheelbase  
- \( W \) = track width  
- \( R \) = turning radius of the vehicle's center

This geometry is difficult to model fully in real-time applications, leading to the use of the **bicycle model**.

---

### Bicycle Model Approximation

The bicycle model simplifies the Ackermann geometry by replacing the two front wheels with a single virtual wheel at the center:

\[
\tan(\delta) = \frac{L}{R}
\]

Where:
- \( \delta \) = steering angle of the virtual front wheel  
- \( R \) = radius of curvature of the center of mass

This model captures the essential motion behavior with fewer parameters and less computational overhead, making it ideal for real-time estimation and control.

---

## 🧮 Mathematical Models

### Non-Holonomic Constraints

Due to the wheel configuration, the vehicle is subject to non-holonomic constraints (i.e., it cannot move sideways). The instantaneous velocity at the center of mass can be described as:

\[
\begin{cases}
\dot{x} = v \cos(\theta) \\
\dot{y} = v \sin(\theta) \\
\dot{\theta} = \frac{v}{L} \tan(\delta)
\end{cases}
\]

Where:
- \( v \) = linear speed  
- \( \theta \) = vehicle orientation  
- \( x, y \) = position in world frame  
- \( \delta \) = steering angle  
- \( L \) = wheelbase

These differential equations describe the **kinematic bicycle model**.

---

## 🤖 ROS Implementation

Two ROS nodes were developed:

### 1. Odometry Node

- Implements the bicycle model equations
- Publishes predicted pose \((x, y, \theta)\) and velocity
- Uses encoder inputs (speed, steering angle)

### 2. GPS Odometry Node

- Converts raw GPS coordinates (latitude, longitude, altitude) into Cartesian coordinates in a fixed local frame (e.g., UTM)
- Publishes GPS position as a `nav_msgs/Odometry` message

---

## 📈 Sensor Fusion with EKF

An Extended Kalman Filter node was implemented to fuse:

- **Predicted pose** from the odometry node
- **GPS position** from the GPS odometry node

This fusion helps correct drift due to accumulated error in dead-reckoning and improves accuracy in the presence of sensor noise.

### EKF Correction Step:

\[
x_t = x_t^- + K_t (z_t - H x_t^-)
\]

Where:
- \( x_t^- \) = predicted state
- \( z_t \) = observed measurement (GPS)
- \( H \) = observation model
- \( K_t \) = Kalman gain

---

## ✅ Output

- `odom_combined`: Corrected odometry published as a `nav_msgs/Odometry` message
- Visualization in RViz for trajectory evaluation
- Logging tools for offline comparison between GPS, odometry, and EKF-based pose

---

## 🧰 Dependencies

- ROS 1 (Noetic recommended)
- `robot_localization` (for EKF)
- `gps_common`
- `geodesy`
- `tf`, `nav_msgs`, `sensor_msgs`

---

## 📌 References

- [Robotics: Modelling, Planning and Control – Siciliano et al.]
- [ROS Navigation Stack]
- [Ackermann Steering Geometry (Technical Notes)]

---

## 📤 Contact

For any questions or collaboration, feel free to reach out to:

**Lorenzo Ortolani**  
📧 [ortolore@gmail.com](mailto:ortolore@gmail.com)  
🔗 [linkedin.com/in/lorenzo-ortolani](https://www.linkedin.com/in/lorenzo-ortolani-6135b7240/)


