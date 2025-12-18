# Chapter 9: Summary & Review Questions

## Recap: The Five Pillars of Humanoid Architecture

1. **Mechanical Structure** (DOF, kinematics, actuator choice) defines *what* the robot can do — reach, grasp, run, jump
2. **Sensors** (vision, IMU, proprioception, F/T) provide the feedback needed to execute movements and react to the environment
3. **Computation** (CPU for real-time control, GPU for perception, edge AI decisions) processes sensor data and computes actions
4. **Software** (ROS 2, real-time middleware, feedback control loops) orchestrates the entire system
5. **Safety** (redundancy, fail-safes, monitoring) protects against failures and keeps humans safe

No single pillar is sufficient. A robot with perfect control algorithms but weak actuators cannot perform dynamic tasks. A robot with powerful actuators but no sensor feedback cannot balance or grasp. A robot with perfect sensors and computation but no safety redundancy is dangerous.

## Connection to Module 2: Simulation as Architecture Validation

In Module 2, you built digital twins in Gazebo and learned to optimize controller parameters in simulation. Those simulations were constrained by the same physics that constrain real robots: gravity, friction, contact forces, actuator limits. By validating control algorithms in a physics-faithful simulator, you learned to respect hardware constraints *before* testing on real machines.

**Key insight**: The architecture decisions in this module (DOF, actuator power, sensor latency, compute budget) are *constraints* that your controllers in Module 4 must satisfy. A control algorithm that assumes instant communication or infinite compute power will fail on real hardware.

## Connection to Module 4: Control Algorithms Within Hardware Constraints

Module 4 will teach inverse kinematics (computing joint angles to reach a target), trajectory planning (smooth paths that respect joint limits), and feedback control (PID, LQR, MPC). All of these algorithms must run **within the hardware constraints** established here:
- Joint angle limits, velocity limits, torque limits
- Sensor latencies (vision 33 ms, IMU 10 ms, joint encoders 5 ms)
- Control loop frequency (can we afford 1000 Hz or only 200 Hz?)
- Power budget (expensive algorithms must run less frequently)

A control algorithm that commands a joint to rotate 1000°/s when the servo can only reach 100°/s will fail. An algorithm that requires camera images every 1 ms but cameras only update every 33 ms will fail. Design these constraints out early by understanding the architecture.

## Connection to Module 5: AI Integration Within Power and Latency Budgets

Module 5 will explore learning-based perception (CNN object detection), learning-based control (neural network policies), and sim-to-real transfer. All of these consume compute:

- Training a deep neural network requires 100s of hours of GPU compute
- Running inference on a CNN costs 0.5–5 Watts per image
- Running an LSTM (recurrent network) for state estimation costs 1–10 Watts

The architecture in this module sets the **power budget**: if the GPU consumes 50 W and the battery has 5 kWh, and the robot runs 8 hours, then AI algorithms can consume at most 50 W × 8 h / 5 kWh = 10% of energy. This forces **strategic choices**: run expensive models infrequently (e.g., object detection once per second, not per frame), use efficient architectures (MobileNets instead of ResNet-152), or offload to the cloud.

By understanding the hardware constraints in Module 3, you will make better architectural choices in Module 5.

---

## Review Questions and Self-Assessment

Use these questions to test your understanding of humanoid robot architecture and integration. Questions are designed to test *integration* — connecting multiple subsystems — rather than isolated facts. **You should be able to answer most of these using only the chapter content.**

### Mechanical Structure and Kinematics (Questions 1–2)

**Q1: Degrees of Freedom Trade-Off**
Explain why Boston Dynamics Atlas has 28 DOF rather than either 8 DOF (minimum for bipedal walking) or 100+ DOF (approaching human dexterity). What constraints does each extreme create for control? How does 28 DOF balance these constraints?

*Hint: Consider control computation, reachability, dexterity, and actuator requirements.*

**Q2: Kinematic Constraints and Control**
The human knee is a hinge joint (1 DOF: flexion/extension only). How does this mechanical constraint shape the control problem for walking? What would change if the knee had 2 DOF (pitch + roll)? Would that be "better" for control?

*Hint: Think about the number of variables to control and the ability to generate desired foot trajectories.*

### Sensors and Sensor Fusion (Questions 3–4)

**Q3: Vision vs. Proprioception in Walking**
During walking on uneven terrain, which sensory modality is more critical: vision (detecting the terrain ahead) or proprioception + IMU (detecting the robot's own body state)? Why? What would happen if the robot lost vision vs. lost IMU?

*Hint: Consider latency and feedback timing. Vision updates at 30 Hz; IMU at 100+ Hz.*

**Q4: Sensor Fusion and Graceful Degradation**
Boston Dynamics Atlas can walk outdoors without GPS or external sensors. If one sensor fails (e.g., one camera or one IMU), how might the robot compensate? What is the fundamental limit of degradation — which sensor cannot fail without causing catastrophic loss of capability?

*Hint: What is the minimal sensor set needed to stand up, balance, and walk slowly?*

### Actuators and Power (Questions 5–6)

**Q5: Hydraulic vs. Electric Trade-Off**
Boston Dynamics Atlas uses hydraulic actuators, while most smaller humanoid robots (e.g., Honda ASIMO's successors) use electric motors. Why did Atlas choose hydraulics despite their complexity? What would an electric-motor version of Atlas sacrifice?

*Hint: Consider power density, speed, load capacity, and control complexity.*

**Q6: Energy Budget and Activity Duration**
If Boston Dynamics Atlas has a 80 kWh battery and peak power draw is 80 kW (during jumping), how long can it jump continuously? If the robot operates at 10 kW average during walking, how much time does it have for exploring tasks?

*Hint: Energy = Power × Time. But peak power is different from average power.*

### Compute and Real-Time Systems (Question 7)

**Q7: Real-Time Control Loop Design**
Sketch the sequence of events in a control loop for humanoid walking:
1. Sensor reads (IMU, encoders, cameras)
2. Sensor fusion (where does this happen?)
3. Control computation (what does this compute?)
4. Motor commands (what is sent to actuators?)
5. Next control cycle

How long does each step take? If the target control frequency is 200 Hz, what is your latency budget for each step?

### Safety and Redundancy (Question 8)

**Q8: Failure Modes and Recovery**
During a walking gait, Atlas' IMU suddenly stops reporting data (sensor failure). How does the robot detect this failure? What is the safe state? Can the robot recover from this failure, or must it fall?

*Hint: Think about watchdog timers and sensor monitoring.*

### System Integration (Questions 9–10)

**Q9: Design Decision Traceability**
Choose one observable capability of Boston Dynamics Atlas (e.g., "can jump 1.5 m while maintaining balance" or "can pick up objects it has never seen"). Trace this capability back to at least 3 architectural decisions (mechanical, sensory, computational, or safety). Explain how each decision enables or constrains the capability.

*Hint: Use the case study section as a guide.*

**Q10: Architecture for a New Application**
Imagine designing a humanoid robot for underwater inspection (exploring sunken shipwrecks, identifying rust/corrosion). How would the architecture differ from Boston Dynamics Atlas (built for dynamic locomotion in air)?

Consider:
- **Mechanical**: Would you want the same 28-DOF structure? Why/why not?
- **Sensors**: How would underwater sensing differ? (Sonar vs. vision, pressure vs. air IMU)
- **Actuators**: Are hydraulics still optimal? What about electric motors with pressure-resistant housings?
- **Compute**: What latency requirements would change?
- **Power**: Can you use tethered power instead of batteries?

*This question asks you to think beyond Atlas and apply the architectural principles to a novel scenario.*

---

## References

Qiu, Y., Zhang, Y., Huang, Z., Liu, H., & Hu, Y. (2024). Deep Reinforcement Learning for Sim-to-Real Transfer in a Humanoid Robot Barista. *2024 IEEE-RAS 23rd International Conference on Humanoid Robots (Humanoids)*, 1–8. IEEE. https://ieeexplore.ieee.org/document/10907454/

Tobin, J., Fong, R., Ray, A., Schneider, J., Zaremba, W., & Abbeel, P. (2017). Domain randomization for transferring deep neural networks from simulation to the real world. *2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 23–30. IEEE. https://doi.org/10.1109/IROS.2017.8202133

Wang, X., Guo, W., Zhang, T., Lu, Z., & Zhao, M. (2025). Robust Dynamic Walking for Humanoid Robots via Computationally Efficient Footstep Planner and Whole-Body Control. *Journal of Intelligent & Robotic Systems*, Vol. 111, Article 49. Springer. https://doi.org/10.1007/s10846-025-02249-w
