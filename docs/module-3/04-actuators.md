# Chapter 4: Actuators & Power

A robot's mechanical structure defines *what* it can do. Actuators define *how much* power it has to do it. Power budget determines *how long* it can operate before the battery depletes.

## Actuation Technologies: Electric vs. Hydraulic

**Electric Motors** (brushless DC, servo motors)
- **Advantages**: Simple control (just a voltage command), efficient energy conversion (~90%), silent operation, lightweight
- **Disadvantages**: Lower power density (~200 W/kg), limited torque at low speeds
- **Use case**: Robots optimized for efficiency and quiet operation (indoor collaborative robots, humanoids designed for human spaces)

**Hydraulic Actuators**
- **Advantages**: Very high power density (~1000 W/kg), excellent for explosive movements, strong force capability, responsive (high-pressure fluid transmits force instantly)
- **Disadvantages**: Complex control (must manage pressure pumps and valving), noisy (pump compressor), lower efficiency (~60%), messy (potential fluid leaks)
- **Use case**: Robots requiring raw power (heavy lifting, dynamic running, jumping)

Boston Dynamics Atlas uses **hydraulics** because it prioritizes dynamic capability: jumping, climbing stairs, running at 1.6 m/s, lifting 45 kg. An electric-motor version of Atlas would be heavier, slower, and unable to generate the explosive forces needed for these behaviors. (Alfayad et al., 2016)

## Series Elastic Actuators (SEA): Compliance for Force Control

Standard stiff actuators (direct motor-to-joint connection) produce very high peak forces but are difficult to control precisely. **Series Elastic Actuators (SEA)** insert a spring (mechanical compliance) between the motor and the joint:

**Why springs?**
- **Force sensing**: Spring deflection = applied force. The motor doesn't need a separate torque sensor; it can estimate joint torque from spring compression
- **Compliance**: Spring acts as a shock absorber. If the robot bumps into an obstacle, the spring deforms rather than the joint breaking
- **Energy recovery**: Spring stores energy during a collision and returns it (like a pogo stick), improving efficiency

Series Elastic Actuators were pioneered by Pratt and Williamson at MIT (Pratt & Williamson, 1995) and are now standard in advanced humanoids. **Boston Dynamics Atlas uses SEA in critical joints (hip, knee) to enable compliant walking and stable manipulation.** (Delgado-Gonzaga et al., 2024)

Example: Grasping with force feedback
- Robot approaches object with gripper
- As gripper closes, the spring compresses as force increases
- When spring deflection reaches 50 N, motor stops commanding closure
- Robot maintains 50 N grip: strong enough not to slip, gentle enough not to crush
- If object shifts, spring deflection changes, and control loop adapts

## Power Management and Energy Budgets

A 28-DOF humanoid is a power-hungry system. Atlas operates on a battery, so energy efficiency matters. Power budget breaks down as:

1. **Hydraulic pump**: ~40% of power (driving the high-pressure pump)
2. **Actuators (motors/hydraulics)**: ~35% of power (moving the joints)
3. **Onboard compute (CPU/GPU)**: ~15% of power (running perception and control)
4. **Sensors and electronics**: ~10% of power

Atlas' battery allows roughly **1–2 hours of continuous operation** at moderate activity levels. Running, jumping, or heavy manipulation drains the battery faster. (Boston Dynamics, 2023)

**Key insight for Module 4 (Control) and Module 5 (AI)**: Control algorithms must be energy-efficient. Running expensive deep learning models continuously would drain the battery. Module 5 will explore strategies like **selective perception** (only run expensive models when needed) and **edge AI** (lightweight models onboard, expensive models in the cloud).

---

## References

Alfayad, S., Ouezdou, F. B., Namoun, F., Bruneau, O., & Henaff, P. (2016). Development of a fast torque-controlled hydraulic humanoid robot that can balance compliantly. *2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids)*, 673–680. IEEE. https://ieeexplore.ieee.org/document/7363420/

Boston Dynamics. (2023). Atlas Robot Specifications and Documentation. Retrieved from https://www.bostondynamics.com/

Delgado-Gonzaga, J., Lee, S., & Sentis, L. (2024). Design of a Series-Elastic Actuator for a Humanoid Robot for Space Applications. *2024 IEEE-RAS 23rd International Conference on Humanoid Robots (Humanoids)*, 1–7. IEEE. https://ieeexplore.ieee.org/document/10668290/

Pratt, G. A., & Williamson, M. M. (1995). Series elastic actuators. *Proceedings of the 1995 IEEE/RSJ International Conference on Intelligent Robots and Systems*, Vol. 1, 399–406. IEEE. https://ieeexplore.ieee.org/document/525827/
