# Content Outline: Module 3 - Humanoid Robot Architecture

**Purpose**: Define chapter structure, learning progression, word count targets, and FR/SC mappings
**Created**: 2025-12-18
**Target**: 3,000–4,000 words | ≥40% peer-reviewed citations | 4+ Mermaid diagrams | 8–10 review questions

---

## Chapter Structure (10 Sections)

### Section 1: Learning Objectives (100 words)
**Task**: T201 [US1]
**Purpose**: Define specific, measurable learning outcomes aligned with FR-001 through FR-004
**Key Points**:
- 5–6 objectives mapping to section 2–5 content
- Each objective linked to 1–2 review questions
- Accessible to advanced undergraduates with Module 2 background

**Requirements Mapped**:
- FR-001, FR-002, FR-003, FR-004 (foundational)
- SC-001 (word count), SC-007 (Module 2 foundation)

---

### Section 2: Introduction: From Simulation to Physical Robots (200 words)
**Task**: T212 [US1]
**Purpose**: Bridge Module 2 (simulation) to Module 3 (physical architecture)
**Key Points**:
- Recap Module 2 (digital twins, Gazebo simulation, sim-to-real gaps)
- Introduce architecture components and their interactions
- State goal: Understand how hardware and software integrate

**Requirements Mapped**:
- SC-007 (Module 2 integration)
- FR-001 through FR-004 preview

---

### Section 3: Mechanical Structure & Kinematics (400 words)
**Task**: T202–T204 [P] [US1]
**Purpose**: Explain humanoid morphology and kinematic constraints
**Key Points** (with Boston Dynamics Atlas example):
- DOF (degrees of freedom) and kinematic chains
- Mechanical structure → control algorithm constraints
- Case study: Atlas 28 DOF hydraulic structure
- How morphology affects control bandwidth, speed, force capacity

**Visual Assets**:
- Mermaid Diagram 1: Hardware-Software Integration (system block diagram)

**Citation Targets**:
- 2–3 IEEE papers on humanoid kinematics
- Boston Dynamics technical specifications

**Requirements Mapped**:
- FR-001 (mechanical structure, DOF, kinematic chains, control effects)
- SC-001 (word count: 400)
- SC-003 (diagram 1)
- SC-004 (peer-reviewed sources)
- SC-005 (Atlas example: how 28 DOF enables walking + manipulation)

---

### Section 4: Sensors & Proprioception (600 words)
**Task**: T205–T207 [P] [US1]
**Purpose**: Explain how sensors enable robot understanding of body and environment
**Key Points** (with fusion architecture):
- Vision: RGB-D cameras, stereo, onboard CNN processing
- Vestibular: IMU (gyroscope, accelerometer) for orientation tracking
- Proprioceptive: Joint encoders, motor current feedback (backdrivability of SEA)
- Force/Torque: End-effector and joint F/T sensors for interaction control
- Tactile: Skin pressure sensors (Boston Dynamics Atlas R3)
- Sensor Fusion: EKF/UKF asynchronously fusing multimodal data

**Visual Assets**:
- Mermaid Diagram 2: Sensor-Actuator Data Flow (perception → state estimation → control → actuation → feedback with latencies)

**Citation Targets**:
- 3–4 papers (vision sensors, fusion algorithms, humanoid sensing)
- Boston Dynamics sensor specifications

**Requirements Mapped**:
- FR-002 (4+ sensor modalities, proprioception, interaction, awareness roles)
- SC-001 (word count: 600)
- SC-003 (diagram 2)
- SC-004 (peer-reviewed sources on sensor fusion)

---

### Section 5: Actuators & Power Delivery (500 words)
**Task**: T208–T209 [P] [US1]
**Purpose**: Explain actuation technologies and power budget constraints
**Key Points** (with explicit trade-offs):
- Electric motors: Efficiency, silent operation, control complexity, speed/torque
- Hydraulic actuators: Power density (~1000 W/kg), response time, heat dissipation
- Series Elastic Actuators (SEA): Compliance, force control, energy absorption (Pratt et al.)
- Transmission mechanisms: Gear ratios, bandwidth constraints, power loss
- Power management: Battery capacity, thermal management, operational duration

**Case Study**: Boston Dynamics Atlas uses hydraulics for 45 kg lifting capacity; why this choice?

**Citation Targets**:
- 3–4 papers (actuator design, SEA, hydraulic systems)
- Boston Dynamics power budget specifications

**Requirements Mapped**:
- FR-003 (actuator technologies, electric/hydraulic/SEA, trade-offs)
- FR-006 (power management impact on compute and duration)
- SC-001 (word count: 500)
- SC-004 (peer-reviewed sources on actuation)
- SC-005 (Atlas choice: hydraulics for power density to enable manipulation)

---

### Section 6: Hardware Compute & Real-Time OS (400 words)
**Task**: T210–T211 [P] [US1]
**Purpose**: Explain compute architecture and real-time constraints
**Key Points**:
- CPU for real-time control loops (hard deadlines, 100–1000 Hz)
- GPU for perception (vision CNNs, sensor fusion acceleration)
- Edge AI trade-offs: Onboard GPU (low latency, high power) vs. cloud (lower power, higher latency, connectivity dependent)
- Real-time OS: ROS 2 with DDS, cycloneDDS, real-time QoS settings
- Latency budgets and control frequency matching

**Citation Targets**:
- 2–3 papers (real-time OS, embedded systems, ROS 2 middleware)
- ROS 2 official documentation

**Requirements Mapped**:
- FR-004 (CPU, GPU, edge AI placement, justified rationale)
- SC-001 (word count: 400)
- SC-004 (peer-reviewed sources on real-time systems)

---

### Section 7: ROS 2 Software Stack (600 words)
**Task**: T301–T304 [P] [US2]
**Purpose**: Explain real-time software integration patterns (GRADUATE LEVEL)
**Key Points**:
- Node architecture: Sensor drivers → fusion → planning → control → actuators
- Real-time pub-sub patterns: DDS QoS, message latency, frequency matching
- Sensor-actuator feedback loops (closed-loop control)
- Integration with Gazebo simulator (Module 2 testing framework)
- Concrete example: Walking control pipeline (perception → balance → step planning → leg servo)

**Visual Assets**:
- Mermaid Diagram 3: ROS 2 Node Architecture (sensor_drivers → sensor_fusion_node → state_estimator → planning_layer → control_layer → motor_drivers)
- Pseudocode: ROS 2 1000 Hz control servo loop example

**Citation Targets**:
- 1–2 ROS 2 papers/documentation on real-time middleware
- Boston Dynamics control architecture (if public)

**Requirements Mapped**:
- FR-005 (ROS 2 stack, real-time constraints, node architecture, sensor-actuator integration)
- SC-001 (word count: 600)
- SC-003 (diagram 3)
- SC-004 (peer-reviewed sources on ROS 2 real-time)

---

### Section 8: Safety, Redundancy & Integration (400 words)
**Task**: T305–T307 [P] [US2]
**Purpose**: Explain how safety integrates across all subsystems (GRADUATE LEVEL)
**Key Points**:
- Mechanical safeguards: Stops, brakes, over-torque protection
- Electrical redundancy: Dual motor drives, power system monitoring
- Software monitoring: Watchdogs, safe state detection, emergency stop
- Graceful degradation: Sensor failure modes and recovery
- System-level integration: How safety layers interact across mechanics, sensors, compute

**Visual Assets**:
- Mermaid Diagram 4: Compute & Hardware Placement (CPU/GPU distribution showing hard-real-time control on CPU, perception on GPU, edge AI decision tree)

**Citation Targets**:
- 1–2 safety papers (ISO 13849, robotics safety architecture)

**Requirements Mapped**:
- FR-007 (safety mechanisms, redundancy, fail-safes, integration)
- SC-001 (word count: 400)
- SC-003 (diagram 4)
- SC-004 (peer-reviewed sources on safety)

---

### Section 9: Case Study - Boston Dynamics Atlas (700 words)
**Task**: T401–T402 [US3]
**Purpose**: Connect architecture decisions to observable robot capabilities
**Key Points** (with explicit decision-to-behavior mapping):
- History and design philosophy (DARPA challenges → commercial platform)
- Mechanical architecture: 28 DOF, hydraulic transmission, gear ratios
- Sensor suite: Cameras, IMUs, F/T sensors, joint torque feedback, tactile network
- Compute architecture: Onboard GPU, ROS 2 control integration
- Observable capabilities: Walking, climbing, object manipulation, dynamic balance recovery
- **Design-to-Behavior Mapping** (5+ decisions):
  1. **28 DOF mechanical structure** → enables walking AND manipulation (many smaller joints = fine control)
  2. **Series Elastic Actuators** → enable compliant hand manipulation (force feedback allows closed-loop grip adjustment)
  3. **Hydraulic actuation** → provides 1000 W/kg power density, enabling 45 kg lifting capacity and explosive movements
  4. **Real-time control loop @ 200 Hz** → allows dynamic balance recovery during walking perturbations
  5. **Sensor fusion (camera + proprioceptive + IMU)** → enables stable walking on uneven terrain without external sensors

**Citation Targets**:
- 3–4 Boston Dynamics papers/reports
- 2–3 IEEE papers citing Atlas architecture

**Requirements Mapped**:
- FR-009 (detailed case study, design decisions → observable capabilities)
- SC-001 (word count: 700)
- SC-004 (all claims sourced)
- SC-005 (5+ decision-to-behavior mappings with citations)

---

### Section 10: Summary, Review Questions & References (500 words)

#### Summary (200 words)
**Task**: T406 [US3]
**Purpose**: Recap architecture and connect to learning progression
**Key Points**:
- Recap core concepts: Mechanics, sensors, actuators, compute, integration, safety
- Explicit connection to Module 2: Simulation → real-world hardware constraints
- Forward references to Module 4: Control algorithms must respect hardware latencies/constraints
- Forward references to Module 5: AI training must account for power budgets, edge computing
- Key insight: Architecture enables or constrains capabilities

**Requirements Mapped**:
- SC-007 (Module 2 integration, Module 4–5 forward prep)

#### Review Questions (8–10, ~300 words)
**Task**: T404–T405 [US3]
**Purpose**: Test architecture integration (not isolated subsystems)
**Question Distribution**:
- Q1–Q2: Mechanics & kinematics (5 constraints, walking vs. manipulation trade-offs)
- Q3–Q4: Sensors & fusion (vision + proprioception necessity, failure recovery)
- Q5–Q6: Actuators & power (hydraulic vs. electric comparison, power constraints)
- Q7: Compute & ROS 2 (sketch node architecture, identify real-time constraints)
- Q8: Safety (IMU failure during balance, system recovery)
- Q9–Q10: Integration (hardware-software interaction in case study)

**Requirements Mapped**:
- FR-010 (8–10 review questions, integration concepts)
- SC-002 (8/10 answerable from chapter content)

#### References (APA formatted)
**Task**: T521 [Phase 6]
**Purpose**: Bibliography with ≥40% peer-reviewed
**Target**: 10–12 peer-reviewed (IEEE, Springer, ACM); ≤20% industry sources

**Requirements Mapped**:
- SC-001 (≥40% peer-reviewed citations)
- SC-004 (all technical claims sourced)

---

## Word Count Targets & Validation

| Section | Target (words) | Task | Requirements |
|---------|---|---|---|
| 1. Learning Objectives | 100 | T201 | FR-001–004 |
| 2. Introduction | 200 | T212 | SC-007 (Module 2 link) |
| 3. Mechanics | 400 | T202–T204 | FR-001, SC-003 (diagram 1) |
| 4. Sensors | 600 | T205–T207 | FR-002, SC-003 (diagram 2) |
| 5. Actuators & Power | 500 | T208–T209 | FR-003, FR-006 |
| 6. Compute | 400 | T210–T211 | FR-004 |
| 7. ROS 2 Software | 600 | T301–T304 | FR-005, SC-003 (diagram 3) |
| 8. Safety & Integration | 400 | T305–T307 | FR-007, SC-003 (diagram 4) |
| 9. Case Study | 700 | T401–T402 | FR-009, SC-005 |
| 10. Summary & Review | 500 | T404–T406, T521 | FR-010, SC-002, SC-007 |
| **TOTAL** | **3,900** | | **All FRs & SCs** |

---

## Visual Assets Checklist (SC-003: 4+ Mermaid Diagrams)

- [ ] Diagram 1: Hardware-Software Integration (system block diagram) — Task T203
- [ ] Diagram 2: Sensor-Actuator Data Flow (real-time control loop with latencies) — Task T207
- [ ] Diagram 3: ROS 2 Node Architecture (sensor → fusion → planning → control) — Task T302
- [ ] Diagram 4: Compute & Hardware Placement (CPU/GPU distribution) — Task T307

---

## Citation Tracking (SC-001: ≥40% peer-reviewed)

**Targets**:
- Peer-reviewed (IEEE, Springer, ACM, 2018–2025): 10–12 minimum
- Industry sources (Boston Dynamics, ROS 2 docs, datasheets): ≤20% of total

**By Section**:
- Section 3 (Mechanics): 2–3 IEEE papers
- Section 4 (Sensors): 3–4 papers (vision, fusion, sensing)
- Section 5 (Actuators): 3–4 papers (actuators, SEA, Boston Dynamics)
- Section 6 (Compute): 2–3 papers (real-time OS, embedded systems)
- Section 7 (ROS 2): 1–2 papers/docs (real-time middleware)
- Section 8 (Safety): 1–2 papers (safety standards, architecture)
- Section 9 (Case Study): 3–4 Boston Dynamics papers + 2–3 citing Atlas

**Final Audit** (Task T506, Phase 6): Verify ≥40% peer-reviewed ratio

---

## Requirements Mapping Matrix

| FR | Section | Task | Word Count | Diagram | Citations |
|---|---|---|---|---|---|
| FR-001 | Mechanics | T202–204 | 400 | Yes (D1) | 2–3 |
| FR-002 | Sensors | T205–207 | 600 | Yes (D2) | 3–4 |
| FR-003 | Actuators | T208–209 | 500 | — | 3–4 |
| FR-004 | Compute | T210–211 | 400 | — | 2–3 |
| FR-005 | ROS 2 | T301–304 | 600 | Yes (D3) | 1–2 |
| FR-006 | Actuators/Power | T208 | [integrated] | — | [FR-003] |
| FR-007 | Safety | T305–307 | 400 | Yes (D4) | 1–2 |
| FR-008 | All | T203, T207, T302, T307 | — | All 4 diagrams | — |
| FR-009 | Case Study | T401–402 | 700 | — | 3–4 Boston Dynamics + 2–3 IEEE |
| FR-010 | Review Questions | T404–405 | 300 | — | — |

---

## Success Criteria Mapping

| SC | Requirement | Validation Task |
|---|---|---|
| SC-001 | 3,000–4,000 words, ≥40% peer-reviewed | T505, T506 |
| SC-002 | 8/10 review questions answerable from content | T510 |
| SC-003 | 4+ Mermaid diagrams | T508 |
| SC-004 | All claims cited and fact-checked | T507 |
| SC-005 | 5+ design-to-behavior mappings in case study | T509 |
| SC-006 | Academic tone, ≤1% plagiarism | T511, T512 |
| SC-007 | Module 2 integration, Module 4–5 prep | T513 |
