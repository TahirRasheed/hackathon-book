# Module 1: The Robotic Nervous System (ROS 2)

## Overview

This module teaches **ROS 2 middleware** as the nervous system connecting AI agents to humanoid robots. Over three chapters, students progress from understanding ROS 2's conceptual role to writing Python agents that communicate with robot hardware via standardized message patterns.

## Chapter Structure

### Chapter 1: Introduction to ROS 2 and the Robotic Nervous System
**File**: `00-intro.md`

**Goal**: Students understand ROS 2 as middleware bridging AI and humanoid robotics.

**Topics**:
- What is middleware and why robots need it
- Distributed systems in humanoid control
- ROS 2's role as the "robotic nervous system"
- Three real-world humanoid use cases (Spot, Optimus, HSR)

**Learning Outcomes**:
- Explain how ROS 2 enables AI agents to coordinate with robot hardware
- Map a humanoid's sensorimotor pipeline onto a distributed ROS 2 node topology
- Identify which ROS 2 communication patterns suit different control scenarios

---

### Chapter 2: ROS 2 Communication — Nodes, Topics, and Services
**File**: `01-communication.md`

**Goal**: Students master topics (pub-sub), services (request-reply), and message flow in ROS 2.

**Topics**:
- ROS 2 nodes as distributed computing units
- Topics for asynchronous pub-sub communication
- Services for synchronous request-reply patterns
- Message types and quality-of-service (QoS)
- Practical examples: obstacle detection, grasp commands, motor feedback
- Perception → Planning → Action pipeline for humanoids

**Learning Outcomes**:
- Draw communication diagrams for humanoid control tasks
- Explain when to use topics vs services
- Trace message flow through multi-node systems
- Design communication topologies for humanoid robots

---

### Chapter 3: Python AI Agents, rclpy, and Humanoid Descriptions (URDF)
**File**: `02-python-agents.md`

**Goal**: Students write Python ROS 2 agents using rclpy and understand robot descriptions via URDF.

**Topics**:
- Introduction to rclpy (ROS 2 Python client library)
- Writing publishers and subscribers in Python
- Calling services from Python nodes
- URDF (Unified Robot Description Format) XML structure
- Parsing URDF files to understand humanoid morphology
- Integrating AI planning with ROS 2 communication

**Learning Outcomes**:
- Write a functional ROS 2 publisher or subscriber in Python
- Parse URDF files and identify joint hierarchies and links
- Design Python nodes integrating planning logic with ROS 2 communication
- Understand how AI agents read robot descriptions and issue commands

---

## Setup Prerequisites

Before reading any chapter, ensure your ROS 2 environment is ready:

### Option 1: Local Ubuntu Installation
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

### Option 2: Docker
```bash
docker run -it osrf/ros:humble-desktop /bin/bash
```

See the [Quick Start Guide](../../specs/001-ros2-nervous-system/quickstart.md) for detailed setup instructions.

---

## Code Examples

All code examples in this module are located in the `code-examples/` directory:

- `01-publisher.py` — Simple ROS 2 publisher (joint state publisher)
- `01-subscriber.py` — Simple ROS 2 subscriber (joint state consumer)
- `01-service-client.py` — Service client calling a hypothetical grasp service
- `02-urdf-parser.py` — URDF XML parser extracting robot structure
- `02-humanoid-example.urdf` — Sample URDF file for a simple humanoid

All code examples are:
- **Syntactically correct** and can be run directly
- **Embedded in chapter markdown** with full explanations
- **Verified against ROS 2 Humble documentation**
- **Runnable** with a standard ROS 2 installation

---

## Learning Approach

This module follows a **"blackbox then whitebox"** pedagogy:

1. **Blackbox (Chapter 1)**: Understand what ROS 2 does and why humanoid robots need it
2. **Whitebox (Chapter 2)**: Learn how ROS 2 works internally — nodes, topics, services
3. **Hands-On (Chapter 3)**: Write Python code that integrates with ROS 2

---

## Assessment

Each chapter includes:
- **Learning Objectives** (3-5 clear goals at the start)
- **Core Concepts** (progressive explanations with examples)
- **Architecture Views** (diagrams and conceptual models)
- **Practical Context** (real-world humanoid robots and scenarios)
- **Review Questions** (8-15 questions answerable from chapter content)
- **Summary** (key takeaways)

---

## Success Criteria

You will have successfully completed this module when you can:

1. **SC-001**: Explain ROS 2's role as middleware for humanoid robotics (Chapter 1)
2. **SC-002**: Design communication topologies using topics and services (Chapter 2)
3. **SC-003**: Write functional Python ROS 2 nodes with rclpy (Chapter 3)
4. **SC-004**: Parse URDF files and understand robot descriptions (Chapter 3)
5. **SC-005**: Integrate AI planning with distributed robot control (Chapter 3)

---

## Language Support

All three chapters are available in:
- **English** (native)
- **اردو** (Urdu translation)

Switch languages using the language selector in the top navigation bar.

---

## References

All content in this module is verified against:
- [ROS 2 Humble Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design Rationale](https://design.ros2.org/)
- [URDF XML Specification](https://wiki.ros.org/urdf/XML)

---

## Getting Started

Begin with **Chapter 1**: [Introduction to ROS 2 and the Robotic Nervous System](./00-intro.md)

---

**Module Version**: 1.0.0
**Created**: 2025-12-17
**Status**: Complete
