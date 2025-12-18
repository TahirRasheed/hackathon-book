# Data Model & Conceptual Entities: Module 1 — The Robotic Nervous System (ROS 2)

**Date**: 2025-12-17  
**Status**: Phase 1 Output

## Key Entities from Feature Specification

### 1. ROS 2 Node
**Definition**: An autonomous computational unit in the ROS 2 graph that performs a specific function (perception, planning, actuation, or coordination).

**Fields**:
- `node_name` (string): Unique identifier in the ROS 2 graph
- `node_namespace` (string): Logical grouping (e.g., `/robot/perception`)
- `subscriptions` (list of Topics): Inbound data streams
- `publishers` (list of Topics): Outbound data streams
- `services` (list of Service): RPC endpoints
- `parameters` (dict): Configuration key-value pairs

**Relationships**:
- Publishes to → Topic
- Subscribes to → Topic
- Calls → Service
- Is called by → Client Node

**State Machine**:
```
UNCONFIGURED → INACTIVE → ACTIVE → FINALIZED
```

---

### 2. Topic
**Definition**: Named channel for asynchronous, decoupled pub-sub communication; enables multiple publishers and subscribers.

**Fields**:
- `topic_name` (string): Unique name (e.g., `/robot/joint_states`)
- `message_type` (class): Data structure (e.g., `sensor_msgs/JointState`)
- `qos_profile` (QoSProfile): Quality-of-Service settings (reliability, durability, history)
- `publishers` (list of Nodes): Nodes writing to this topic
- `subscribers` (list of Nodes): Nodes reading from this topic

**Relationships**:
- Published by → Node
- Subscribed by → Node
- Carries → Message

**Characteristics**:
- Asynchronous (fire-and-forget)
- One-to-many or many-to-many communication
- Best for sensor streams, telemetry, feedback

---

### 3. Service
**Definition**: Synchronous request-reply communication mechanism; client sends request, server responds with reply.

**Fields**:
- `service_name` (string): Unique name (e.g., `/robot/grasp`)
- `request_type` (class): Input structure (e.g., `GraspRequest`)
- `response_type` (class): Output structure (e.g., `GraspResponse`)
- `server_node` (Node): Node providing the service
- `client_nodes` (list of Nodes): Nodes calling the service

**Relationships**:
- Provided by → Node
- Called by → Node
- Returns → Message (Response)

**Characteristics**:
- Synchronous (blocking until response)
- One-to-one communication
- Best for commands, queries, transactional operations

---

### 4. Message
**Definition**: Typed data structure published on topics or exchanged via services; defines the contract between communicating nodes.

**Fields**:
- `message_name` (string): Fully qualified name (e.g., `sensor_msgs/JointState`)
- `fields` (list): Named attributes with types (int, float, string, arrays, nested messages)
- `serialization` (format): CDR (Common Data Representation) used by DDS

**Examples**:
- `sensor_msgs/JointState`: Joint positions, velocities, efforts
- `geometry_msgs/Twist`: Linear/angular velocity for robot motion
- `std_msgs/String`: Generic string payload

**Relationships**:
- Published via → Topic
- Exchanged via → Service

---

### 5. Publisher
**Definition**: Node role that writes messages to a topic at regular or event-driven intervals.

**Fields**:
- `publisher_name` (string): Reference identifier
- `topic` (Topic): Target topic
- `message_type` (class): Data being published
- `publish_frequency` (Hz): Rate (if periodic)
- `node` (Node): Owner node

**State**:
- Active (publishing)
- Inactive (ready but not publishing)

---

### 6. Subscriber
**Definition**: Node role that receives and processes messages from a topic.

**Fields**:
- `subscriber_name` (string): Reference identifier
- `topic` (Topic): Source topic
- `callback` (function): Handler invoked on message receipt
- `message_type` (class): Data being received
- `node` (Node): Owner node

**State**:
- Active (receiving)
- Inactive (ready but not receiving)

---

### 7. URDF (Unified Robot Description Format)
**Definition**: XML-based file describing a robot's kinematic, dynamic, and visual structure.

**Fields**:
- `robot_name` (string): Robot identifier
- `links` (list): Rigid bodies (base, limbs, sensors, actuators)
- `joints` (list): Connections between links (revolute, prismatic, fixed)
- `inertia` (mass properties): Mass, center of gravity, inertia tensor
- `collision_meshes` (files): 3D geometry for collision detection
- `visual_meshes` (files): 3D geometry for visualization

**Relationships**:
- Robot is composed of → Links
- Links connected by → Joints
- Each Joint defines parent/child link relationship

**Structure Example**:
```xml
<robot name="humanoid">
  <link name="base_link">...</link>
  <link name="left_arm">...</link>
  <joint name="left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_arm"/>
  </joint>
</robot>
```

---

### 8. Humanoid Robot
**Definition**: Mobile manipulator with human-like morphology (head, torso, arms, legs) designed for physical interaction in human environments.

**Fields**:
- `robot_type` (string): e.g., "wheeled_humanoid", "biped_humanoid"
- `degrees_of_freedom` (int): Number of actuated joints
- `payload_capacity` (kg): Maximum gripper/arm load
- `locomotion_speed` (m/s): Max forward speed
- `sensors` (dict): IMU, lidar, cameras, joint encoders
- `actuators` (list): Motors, grippers, legs
- `control_computer` (spec): CPU, RAM, ROS 2 installation

**Relationships**:
- Described by → URDF
- Controlled via → ROS 2 Node Graph
- Publishes state via → Topics
- Receives commands via → Services/Topics

---

## Validation Rules

1. **Node Naming**: Must be unique within namespace; alphanumeric + underscores
2. **Topic Naming**: Follows ROS convention (lowercase, underscores, forward slashes for hierarchy)
3. **Message Type**: Must be defined in package; version stability enforced
4. **QoS Compatibility**: Publishers and subscribers must have compatible QoS policies
5. **Service Contract**: Request and response types must match across all callers
6. **URDF Validity**: Must be parseable XML; all joint references must point to existing links

---

## State Transitions

### Typical Humanoid Perception → Planning → Action Pipeline

```
1. Perception Node (Subscriber)
   - Subscribes to: /camera/rgb_image, /lidar/scan
   - Publishes to: /perception/objects_detected

2. Planning Node (Subscriber + Service Client)
   - Subscribes to: /perception/objects_detected
   - Calls Service: /planner/compute_grasp
   - Publishes to: /planning/trajectory

3. Motor Controller Node (Subscriber + Publisher)
   - Subscribes to: /planning/trajectory
   - Publishes to: /robot/joint_commands
   - Publishes to: /robot/joint_feedback

4. Humanoid Robot (URDF + Actuators)
   - Receives joint_commands via topics
   - Executes motion
   - Publishes joint_states feedback
```

---

## Phase 1 Status

✅ **COMPLETE** — All key entities modeled. Represents all core concepts from Feature Specification. Ready for content generation.
