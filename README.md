# Ackermann Steering Model & Bicycle Approximation

This repository explains and implements vehicle kinematic models used for localization and motion estimation in mobile robotics, specifically:

- The **Ackermann steering model**, which reflects the true geometry of four-wheeled cars.
- The **bicycle approximation**, a simplified version widely used in robotics and autonomous systems for odometry.

The models are implemented in the context of robotic localization using proprioceptive sensors (like wheel encoders and steering angle sensors), and were developed as part of a project at Politecnico di Milano.

---

## Overview

In autonomous mobile robotics, especially for car-like platforms, estimating the vehicle’s pose (position and orientation) over time is essential. When GPS or external localization is not reliable or available, internal odometry models must be used.

Two common approaches are:
- **Ackermann steering geometry**, which accurately models the steering behavior of vehicles with front-wheel steering.
- **Bicycle model approximation**, which simplifies the system to two wheels while preserving the core motion behavior.

---

## Ackermann Steering Model

The Ackermann model is used to describe the real-world behavior of four-wheeled vehicles where only the front wheels are steerable. Each front wheel follows a different path when turning, and the turning radius depends on the vehicle's geometry and the individual steering angles.

This model is highly accurate but computationally complex and difficult to apply directly in real-time systems.

---

## Bicycle Approximation

To make real-time implementation feasible, the bicycle model simplifies the Ackermann configuration by approximating the vehicle as having just two wheels — one steerable front wheel and one fixed rear wheel.

This model is widely used for:
- Odometry and dead-reckoning
- Path planning
- Real-time control and estimation

Despite being a simplification, it provides sufficiently accurate results for many robotics and automotive applications, especially when paired with sensor fusion (e.g., using an Extended Kalman Filter).

---

## Applications

This repository provides an implementation of:
- Odometry estimation using encoder and steering data
- Localization using the bicycle model for position tracking
- ROS-compatible nodes for publishing estimated pose
- Tools for integrating GPS measurements and performing sensor fusion

---

## Tools & Technologies

- ROS 1 (tested with Noetic)
- C++ / Python
- `nav_msgs/Odometry`, `tf`, `geometry_msgs`
- Extended Kalman Filter for improving estimation accuracy

---

## Context

The models and algorithms are derived from the *Robotics* course slides by Prof. Matteo Matteucci at Politecnico di Milano, and were applied in a university project involving vehicle localization on the Monza F1 track.

---

## 📩 Contact

**Lorenzo Ortolani**  
📧 [ortolore@gmail.com](mailto:ortolore@gmail.com)  
🔗 [linkedin.com/in/lorenzo-ortolani](https://linkedin.com/in/lorenzo-ortolani-6135b7240)

---

> This project demonstrates the balance between model accuracy and computational efficiency when working with real-world autonomous systems.
