# Research & Evidence Base: Module 3 - Humanoid Robot Architecture

**Purpose**: Consolidate peer-reviewed evidence for all technical claims in Module 3 chapter
**Created**: 2025-12-18
**Status**: In Progress

---

## Research Priority 1: Boston Dynamics Atlas Architecture

**Goal**: Document mechanical, sensory, actuator, compute, and power specifications for Boston Dynamics Atlas

### Mechanical Structure & Kinematics

**Decision**: 28 DOF hydraulic-actuated humanoid platform

**Evidence Sources**:
- [ ] Source 1: IEEE paper on Atlas mechanical design
- [ ] Source 2: Boston Dynamics technical report

**Key Specifications**:
- DOF count: 28
- Kinematic chains: Upper body, lower body, head
- Gear ratios: [To be filled from sources]
- Weight: [To be filled]
- Height: [To be filled]

---

## Research Priority 2: ROS 2 Real-Time Control Patterns

**Goal**: Document node architecture, latency budgets, DDS QoS settings for humanoid control

### Node Architecture for Control Loops

**Decision**: Pub-sub architecture with sensor drivers → fusion → planning → control → actuators

**Evidence Sources**:
- [ ] Source 1: ROS 2 documentation on real-time middleware
- [ ] Source 2: IEEE paper on humanoid control loops

**Key Specifications**:
- Control loop frequency: 100–1000 Hz
- DDS QoS settings: [To be filled]
- Message latency budgets: [To be filled]

---

## Research Priority 3: Sensor Modalities in Humanoid Context

**Goal**: Document vision, F/T, IMU, proprioceptive, and tactile sensing roles

### Vision Sensors

**Decision**: RGB-D + stereo cameras for perception and navigation

**Evidence Sources**:
- [ ] Source 1: Robot vision sensor specs
- [ ] Source 2: CNN perception papers

**Key Specifications**:
- Camera types: RGB-D (Kinect-style), stereo pairs
- Update frequency: [To be filled]
- Processing: Onboard CNN inference

---

## Research Priority 4: Actuator Trade-Offs

**Goal**: Compare electric, hydraulic, and Series Elastic Actuator technologies

### Hydraulic vs. Electric Actuation

**Decision**: Boston Dynamics Atlas uses hydraulics for power density

**Evidence Sources**:
- [ ] Source 1: Pratt et al. on Series Elastic Actuators
- [ ] Source 2: Boston Dynamics whitepaper on hydraulic actuation
- [ ] Source 3: IEEE paper on actuator trade-offs

**Key Specifications**:
- Power density: Hydraulic ~1000 W/kg, Electric ~200 W/kg
- Response time: [To be filled]
- Control complexity: [To be filled]
- Backdrivability: [To be filled]

---

## Research Priority 5: Compute Architecture & Edge AI

**Goal**: Document CPU/GPU placement, real-time OS, and edge AI trade-offs

### CPU for Real-Time Control

**Decision**: Dedicated CPU core(s) for hard real-time control loops

**Evidence Sources**:
- [ ] Source 1: Real-time Linux documentation
- [ ] Source 2: IEEE paper on embedded real-time control

**Key Specifications**:
- Control loop: Hard real-time, 1000 Hz
- OS: Linux RT or RTOS
- Latency budget: <1 ms per loop

### GPU for Perception

**Decision**: GPU for vision CNN inference and sensor fusion

**Evidence Sources**:
- [ ] Source 1: Embedded GPU specifications
- [ ] Source 2: CNN inference latency studies

---

## Research Priority 6: Safety & Redundancy

**Goal**: Document mechanical, electrical, and software safety mechanisms

### Mechanical Safety

**Decision**: Mechanical stops + brakes + over-torque protection

**Evidence Sources**:
- [ ] Source 1: ISO 13849 safety standards
- [ ] Source 2: Robotics safety design papers

---

## Research Priority 7: Module 2 Integration (Sim-to-Real)

**Goal**: Document Gazebo simulation for testing control architectures

### Simulation for Testing

**Decision**: Gazebo for control architecture testing; physics fidelity critical

**Evidence Sources**:
- [ ] Source 1: Gazebo physics engine documentation
- [ ] Source 2: Sim-to-real transfer papers

---

## Citation Tracking

**Target**: ≥40% peer-reviewed (IEEE, Springer, ACM, 2018–2025)
**Current Count**: 0/0 (In Progress)

### Peer-Reviewed Sources (Target: 10–12)

- [ ] Source 1: [Title, Authors, Venue, Year]
- [ ] Source 2: [Title, Authors, Venue, Year]
- [ ] Source 3: [Title, Authors, Venue, Year]

### Industry Sources (Max 20%)

- [ ] Boston Dynamics Technical Reports: [URLs/Titles]
- [ ] ROS 2 Official Documentation: [URLs/Titles]

---

## Notes

- Research tasks (T101–T127 in tasks.md) will populate these sections
- Each priority area requires 1–2 peer-reviewed sources minimum
- All technical claims in chapter must be traceable to sources listed here
