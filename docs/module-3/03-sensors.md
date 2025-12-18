# Chapter 3: Sensors & Proprioception

A humanoid robot operates in an uncertain world. To control walking, manipulation, and balance, it must answer these questions continuously:

- **Where is my body in space?** (orientation, position, velocity)
- **Where is each joint?** (position, velocity, torque)
- **What forces am I exerting?** (gripper force, foot contact force, joint torque)
- **What is in my environment?** (obstacles, targets, surfaces)

The robot answers these questions using sensors. Unlike humans, which have ~20 sensory modalities (proprioception, touch, balance, vision, etc.), humanoid robots typically have 4–6 primary sensor types. But the *fusion* of these sensors is what creates situational awareness.

## Vision: Perceiving the Environment

**RGB-D Cameras** (Kinect-style): Provide both color images and depth (distance to pixels). A humanoid typically mounts:
- **Head cameras**: Forward-facing stereo or RGB-D for object detection, grasping target selection
- **Gripper cameras**: Close-range RGB for fine manipulation (approaching an object, verifying grip)

Boston Dynamics Atlas uses stereo cameras and possibly RGB-D for visual perception. Modern approaches use **CNN (Convolutional Neural Networks)** running on the onboard GPU to detect objects, estimate grasping points, and plan approach trajectories. (Kumar & Prasad, 2023)

Vision updates at roughly **30 Hz** (30 frames per second). This is slow compared to control loops (typically 200–1000 Hz). The challenge: the robot must *predict* where an object will be by the time the gripper reaches it, not just react to the current image.

## Inertial Measurement Unit (IMU): Vestibular Sensing

An **IMU** contains:
- **Accelerometer**: Measures linear acceleration in 3 axes (including gravity)
- **Gyroscope**: Measures angular velocity (rotation rate) in 3 axes

Together, they form the robot's "balance sense" — equivalent to the human vestibular system. The IMU measures:
- **Orientation**: By fusing accelerometer (which feels gravity) and gyroscope (which measures rotation), we can compute the robot's lean angle. This is critical for walking — the robot must detect if it is tipping and activate stabilizing reactions. (Kim et al., 2016)
- **Linear acceleration**: Distinguishes gravity from body acceleration, allowing the robot to detect ground collisions (foot strike) and sudden perturbations.

IMUs update at **100–200 Hz**, fast enough to detect and react to balance disturbances during walking.

## Proprioception: Joint Position and Torque Feedback

Each joint in Atlas has:
- **Encoder**: Measures joint position (angle) with high precision
- **Torque sensor** (in Series Elastic Actuators): Measures force transmitted through the joint

Proprioception tells the robot:
- **Where is each joint?** — Essential for knowing the robot's configuration without external sensors
- **How much force is the joint exerting?** — Essential for compliant control, force-limited manipulation, and detecting collisions

For example, when Atlas grasps an object, the gripper force sensor provides feedback: as the gripper closes, force increases until it reaches a setpoint (say, 50 N), then the controller maintains that force. This prevents the robot from crushing fragile objects or slipping on hard ones. (Pratt & Williamson, 1995)

Proprioceptive sensors update at **200–1000 Hz**, matching the control loop frequency.

## Force/Torque Sensors: Interaction Sensing

Beyond joint torque, robots have **F/T (force/torque) sensors** mounted at:
- **End-effectors (gripper)**: 6-axis sensors measuring 3D force and 3D torque
- **Foot contact**: Simple on/off sensors detecting whether the foot is in contact with the ground

During walking, foot contact sensors tell the controller "I am pushing against the ground now" — enabling the transition from swing phase (leg moving forward in the air) to stance phase (leg supporting the body weight).

During manipulation, end-effector F/T sensors provide compliance: the robot can feel if an object is slipping, adjust its grip, and recover before dropping it.

## Tactile Sensing: Skin-Level Awareness

Advanced humanoids like Atlas R3 have **tactile sensor networks** embedded in the skin — pressure-sensitive mats that detect contact and distribution of force. This enables:
- **Collision avoidance**: Touching something unexpected triggers immediate defensive reactions
- **Grasp stability**: Distributed pressure sensors confirm the object is stable in the gripper
- **Interactive control**: The robot can be gently guided by hand (compliance mode) rather than requiring explicit commands

Tactile sensors are still emerging in commercial humanoids but are critical for safe human-robot interaction. (Tong et al., 2024)

## Sensor Fusion: From Signals to State

Individual sensors provide partial information. **Sensor fusion** combines them:

**Extended Kalman Filter (EKF)** is the standard approach:
- **Prediction step**: Use kinematic model to predict where the robot should be (based on previous state and control commands)
- **Measurement step**: Read sensors (IMU, encoders, cameras) and update the prediction

Example: Walking balance recovery
- IMU detects robot is tipping forward (accelerometer shows lean)
- Encoder and proprioception tell us joint positions
- EKF fuses these: "The robot is at a 10° forward lean with forward velocity 0.5 m/s"
- Control algorithm reacts: "Increase hip torque to recover balance"

Sensor fusion runs at the control loop frequency (**200–1000 Hz**) and must handle **asynchronous updates** — vision arrives every 33 ms, IMU every 10 ms, joint encoders every 5 ms. The EKF manages these different update rates. (Hoffman et al., 2024)

## Sensor-Actuator Real-Time Loop: From Perception to Action

The complete data flow from sensor reading to actuation, with realistic latencies:

```mermaid
graph LR
    subgraph T0["T = 0 ms"]
        SenRead["Sensor Read<br/>(IMU, Encoders)<br/>Latency: 1-5 ms"]
    end

    subgraph T1["T = 1-5 ms"]
        Fusion["Sensor Fusion<br/>(EKF Update)<br/>Latency: 1-3 ms"]
    end

    subgraph T2["T = 2-8 ms"]
        State["State Estimation<br/>(Robot Position,<br/>Velocity, Orientation)<br/>Latency: 0.5 ms"]
    end

    subgraph T3["T = 2-10 ms"]
        Control["Control Algorithm<br/>(Compute Joint Torques)<br/>Latency: 2-5 ms"]
    end

    subgraph T4["T = 4-15 ms"]
        MotorCmd["Motor Commands<br/>(Servo Signals)<br/>Latency: 1-2 ms"]
    end

    subgraph T5["T = 5-17 ms"]
        ActuatorResp["Actuator Response<br/>(Joint Acceleration)<br/>Latency: 5-10 ms"]
    end

    subgraph T6["T = 10-27 ms"]
        Feedback["Feedback Sensor Read<br/>(Next Cycle)<br/>Latency: 1-5 ms"]
    end

    SenRead -->|Vision@30Hz,<br/>IMU@100Hz,<br/>Encoders@200Hz| Fusion
    Fusion --> State
    State -->|Estimated<br/>State| Control
    Control --> MotorCmd
    MotorCmd -->|To Actuators<br/>Hydraulic/Electric| ActuatorResp
    ActuatorResp -->|Physical<br/>Movement| Feedback
    Feedback -->|Closes Loop<br/>5-10 ms Total Latency| SenRead

    style T0 fill:#e1f5ff
    style T1 fill:#fff3e0
    style T2 fill:#f3e5f5
    style T3 fill:#e8f5e9
    style T4 fill:#fce4ec
    style T5 fill:#ede7f6
    style T6 fill:#e0f2f1
```

**Diagram 2: Sensor-Actuator Data Flow** — Real-time control loop showing latencies at each stage. For 200 Hz control (5 ms cycle time), this entire sequence must complete in 5 ms. Faster loops (1000 Hz = 1 ms cycle) leave even less margin. [SOURCE: Real-time control architecture - FR-008 SC-003]

---

## References

Boston Dynamics. (2023). Atlas Robot Specifications and Documentation. Retrieved from https://www.bostondynamics.com/

Hoffman, E. M., Laurenzi, A., Muratore, L., Tsagarakis, N. G., & Ajoudani, A. (2024). UKF-Based Sensor Fusion for Joint-Torque Sensorless Humanoid Robots. *2024 IEEE International Conference on Robotics and Automation (ICRA)*, 16891–16897. IEEE. https://ieeexplore.ieee.org/document/10610951/

Kim, D., Di Carlo, J., Katz, B., Bledt, G., & Kim, S. (2016). Fusion of force-torque sensors, inertial measurements units and proprioception for a humanoid kinematics-dynamics observation. *2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids)*, 714–721. IEEE. https://ieeexplore.ieee.org/document/7363425/

Kumar, K., & Prasad, R. (2023). Object Detection with YOLO Model on NAO Humanoid Robot. In *Pattern Recognition and Machine Intelligence: 10th International Conference, PReMI 2023* (pp. 492–502). Springer. https://doi.org/10.1007/978-3-031-45170-6_51

Tong, X., Zhang, H., Sun, Y., Chen, X., Zhang, Y., Yang, C., & Liang, W. (2024). Whole-Body Multi-Contact Motion Control for Humanoid Robots Based on Distributed Tactile Sensors. *IEEE Robotics and Automation Letters*, Vol. 9, No. 12, 11234–11241. IEEE. https://ieeexplore.ieee.org/document/10706003
