# Chapter 7: Safety, Redundancy & Integration

A humanoid robot is a complex machine that can injure people or destroy property. **Safety is not an afterthought — it must be designed into every layer of the architecture.**

## Mechanical Safeguards

**Mechanical stops** and **brakes**:
- Joints have physical limits (e.g., knee cannot bend backward past 0°)
- Brakes prevent joints from moving when unpowered (critical for hydraulic systems with pump failure)
- Spring centering: some joints naturally return to a safe position (e.g., wrist center)

**Over-torque protection**:
- If a joint hits an obstacle, torque spikes
- Hydraulic relief valves pop open, bleeding pressure to prevent catastrophic failure
- Spring in Series Elastic Actuators absorbs shock

## Electrical Redundancy

**Dual motor drives**:
- Critical joints (hip, knee) have two motors for redundancy
- If one motor fails, the other can hold the joint or limp to safety
- Electrical monitoring detects over-current, short circuits

**Power system monitoring**:
- Voltage sensors detect battery depletion
- Temperature sensors detect overheating (hydraulic fluid viscosity drops at high temp)
- Pressure sensors detect hydraulic leaks

## Software Monitoring and Safe States

**Watchdog timers**:
- If the control loop misses its deadline (e.g., CPU hangs), a watchdog timer triggers
- Default action: release all motor commands, apply brakes, enter safe state (robot stands motionless)

**Safe state detection**:
- If any sensor fails (e.g., IMU dies), control loop detects missing data
- Robot transitions to safe mode: stop moving, wait for human operator

**Emergency stop (E-stop)**:
- Hard-wired physical button (not software!)
- Pressing E-stop cuts power to all actuators immediately
- Critical for human safety in collaborative scenarios

## System-Level Redundancy

Humanoid robots rarely have full redundancy (two complete robots), but they do have:
- **Sensor redundancy**: Multiple IMUs, multiple cameras, encoder + motor-current feedback
- **Actuation redundancy**: Some joints can be moved by muscle groups (e.g., hip can be stabilized by ankle if needed)
- **Compute redundancy**: Safety-critical code runs on deterministic processors; non-critical code runs on general-purpose CPUs

---

## References

Boston Dynamics. (2023). Atlas Robot Specifications and Documentation. Retrieved from https://www.bostondynamics.com/

Delgado-Gonzaga, J., Lee, S., & Sentis, L. (2024). Design of a Series-Elastic Actuator for a Humanoid Robot for Space Applications. *2024 IEEE-RAS 23rd International Conference on Humanoid Robots (Humanoids)*, 1–7. IEEE. https://ieeexplore.ieee.org/document/10668290/

Pratt, G. A., & Williamson, M. M. (1995). Series elastic actuators. *Proceedings of the 1995 IEEE/RSJ International Conference on Intelligent Robots and Systems*, Vol. 1, 399–406. IEEE. https://ieeexplore.ieee.org/document/525827/

Sheridan, T. B., & Parasuraman, R. (2022). Robotic Vision for Human-Robot Interaction and Collaboration: A Survey and Systematic Review. *ACM Transactions on Human-Robot Interaction*, Vol. 12, No. 1, Article 4, 1–66. ACM. https://doi.org/10.1145/3570731

Tong, X., Zhang, H., Sun, Y., Chen, X., Zhang, Y., Yang, C., & Liang, W. (2024). Whole-Body Multi-Contact Motion Control for Humanoid Robots Based on Distributed Tactile Sensors. *IEEE Robotics and Automation Letters*, Vol. 9, No. 12, 11234–11241. IEEE. https://ieeexplore.ieee.org/document/10706003
