# 🚘 Ackermann Steering Model & Bicycle Approximation

This repository implements and explains the kinematic models used for car-like vehicles, focusing on:

- The **Ackermann steering geometry**, which models the real behavior of four-wheeled vehicles.
- Its simplified form, the **bicycle approximation**, suitable for odometry and localization algorithms.

The project and this README are based on material presented in the *Robotics Course* by Prof. Matteo Matteucci, Politecnico di Milano.

---

## 📐 Ackermann Steering Model

The **Ackermann steering model** captures the geometry of a vehicle with four wheels, where only the front wheels can steer. The rear wheels are fixed and provide traction.

Key characteristics:
- Wheels have limited turning angles.
- Vehicle cannot rotate in place.
- Turning radius is defined by the geometry and steering angles.

### 🔧 Geometry

In Ackermann steering, each front wheel must follow a different radius curve:

\[
\tan(\alpha_L) = \frac{d}{R + b}, \quad \tan(\alpha_R) = \frac{d}{R - b}
\]

Where:
- \( \alpha_L, \alpha_R \) = steering angles for left and right front wheels  
- \( R \) = turning radius (to the ICR — instantaneous center of rotation)  
- \( d \) = wheelbase (distance between front and rear axles)  
- \( b \) = half the track width

Each wheel must rotate about the same **ICC** to avoid slipping, which defines these turning radii.

### 🌀 Angular Velocity

Each wheel contributes to the vehicle’s angular velocity \( \omega \):

\[
\omega = \frac{V_{FL}}{d \cdot \sin(\alpha_L)} = \frac{V_{FR}}{d \cdot \sin(\alpha_R)}
\]

---

## 🚲 Bicycle Approximation

To simplify computation, the **bicycle model** reduces the four-wheel system into a two-wheel model:

- A single front wheel controls the steering (placed at the midpoint of actual front wheels).
- A single rear wheel provides propulsion.

This model preserves the essential behavior while being easier to implement in real-time systems like EKFs or path planners.

### 🧮 Kinematic Equations

Let \( \alpha \) be the steering angle of the virtual front wheel, and \( v \) be the linear velocity:

\[
\begin{aligned}
x_{k+1} &= x_k + v_k T_s \cos(\theta_k) \\
y_{k+1} &= y_k + v_k T_s \sin(\theta_k) \\
\theta_{k+1} &= \theta_k + \omega_k T_s \\
\text{where} \quad \omega_k &= \frac{v_k}{d} \tan(\alpha_k)
\end{aligned}
\]

Where:
- \( (x, y) \) is the position in global frame  
- \( \theta \) is the heading angle  
- \( d \) is the wheelbase  
- \( T_s \) is the sampling time  

This is the model used for **dead-reckoning odometry** in real-world applications such as self-driving cars and autonomous racing.

---

## 📌 Why Use the Bicycle Model?

While the Ackermann model reflects true vehicle behavior, it is more complex to model and simulate due to independent front wheel angles. The bicycle model:
- Approximates real motion with sufficient accuracy
- Requires fewer parameters and sensors
- Enables fast computation for control and localization

---

## 📷 Visual Reference

![Ackermann Geometry](docs/ackermann_diagram.png)  
*Figure: Ackermann steering — wheels turning around a shared ICR*

---

## 🔧 Applications in This Repository

- Odometry estimation using wheel encoder and steering angle
- Localization through integration over time
- ROS-compatible implementation
- Designed for low-slip environments (e.g., asphalt, track)

---

## 🧰 Dependencies

- ROS 1 (tested with Noetic)
- `tf`, `geometry_msgs`, `nav_msgs`
- C++ or Python (depending on your implementation)

---

## 📩 Contact

Lorenzo Ortolani  
📧 [ortolore@gmail.com](mailto:ortolore@gmail.com)  
🔗 [linkedin.com/in/lorenzo-ortolani](https://linkedin.com/in/lorenzo-ortolani-6135b7240)

---

> _"All models are wrong, but some are useful."_ – George Box  
> The bicycle model is one such useful simplification of a very real problem.

