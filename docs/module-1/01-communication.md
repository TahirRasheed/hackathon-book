# Chapter 2: ROS 2 Communication — Nodes, Topics, and Services

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Explain the role of nodes, topics, and services** in ROS 2's distributed communication architecture
2. **Design communication topologies** that connect perception, planning, and action subsystems in humanoid robots
3. **Distinguish between topics (pub-sub) and services (request-reply)** and apply each pattern appropriately
4. **Trace message flow** through multi-node systems to diagnose communication bottlenecks
5. **Apply Quality-of-Service (QoS) policies** to ensure reliable message delivery for time-critical robot control
6. **Handle edge cases** such as node failures, message delays, and multi-agent coordination conflicts

---

## Core Concepts

### What is a Node?

A **ROS 2 node** is an autonomous computational unit in the ROS 2 graph. Every node is a separate process (or thread) that:
- Runs continuously, processing data and making decisions
- Communicates with other nodes via topics (publish-subscribe) and services (request-reply)
- Has a unique name within the ROS 2 network (e.g., `/robot_controller`, `/obstacle_detector`)
- Follows a standardized lifecycle: UNCONFIGURED → INACTIVE → ACTIVE → FINALIZED

**Real-world analogy**: In a humanoid robot's nervous system, each node is like a specialized neural cluster—the perception cluster processes sensor data, the planning cluster reasons about actions, and the motor control cluster executes commands. These clusters communicate via standardized pathways (topics and services).

### Node Namespaces and Naming

ROS 2 uses **namespaces** to organize nodes hierarchically. For example:
- `/robot_1/arm_controller` — A specific controller in the robot_1 namespace
- `/robot_1/perception/obstacle_detector` — Nested namespace for perception subsystems
- `/robot_2/arm_controller` — Same node name in a different robot's namespace (multi-agent coordination)

**Naming conventions**:
- Use `snake_case` for node names (e.g., `joint_trajectory_controller`, `grasping_planner`)
- Use hierarchical namespaces to group related nodes (e.g., `/robot/arm/`, `/robot/legs/`)
- Avoid hardcoded node names; use configuration files to support dynamic deployment

### Node Lifecycle

Every ROS 2 node transitions through four states:

```
UNCONFIGURED → INACTIVE → ACTIVE → FINALIZED
     ↑                                    ↓
     └────────────────────────────────────┘
```

- **UNCONFIGURED**: Node initialized but not yet ready (resources not allocated)
- **INACTIVE**: Node configured but paused (ready to activate on demand)
- **ACTIVE**: Node running, publishing/subscribing, processing messages
- **FINALIZED**: Node shut down, resources released

This lifecycle ensures graceful startup and shutdown—critical for multi-node robot systems where components must be activated in dependency order.

---

## Topics and Publish-Subscribe (Pub-Sub) Communication

### What is a Topic?

A **topic** is a named channel for **asynchronous, one-way** message distribution. Think of it as a bulletin board where:
- **Publishers** post messages to the topic
- **Subscribers** listen and receive copies of those messages
- Multiple publishers can write to the same topic
- Multiple subscribers can read from the same topic
- Publishers and subscribers don't need to know about each other

**Example**: A humanoid robot's joint position publisher continuously broadcasts the current angle of each joint. Any node interested in joint positions—the planner, the safety monitor, the data logger—can subscribe without the publisher knowing or caring.

### Pub-Sub Communication Flow

```
Publisher (e.g., Joint State Sensor)
         ↓
    Topic: /robot/joint_states
         ↙      ↓      ↘
    Planner  Monitor  Logger
   (Subscriber 1) (Subscriber 2) (Subscriber 3)
```

**Key advantages of pub-sub**:
- **Decoupling**: Publishers and subscribers are independent; new subscribers can join at any time
- **Scalability**: One publisher can serve many subscribers with minimal overhead
- **Asynchronous**: Senders don't wait for receivers to process; messages are queued

### Message Types and Examples

ROS 2 defines standardized message types for common robotics tasks:

#### **sensor_msgs/JointState**
Published by joint sensors or state estimators. Contains joint positions, velocities, and efforts.

```yaml
# Structure
header:
  seq: 1
  stamp:
    sec: 1702881234
    nsec: 567890123
  frame_id: "robot_base"
name: ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "wrist_3"]
position: [0.0, 1.57, -1.57, 0.0, 0.0, 0.0]      # Radians
velocity: [0.1, 0.05, -0.08, 0.0, 0.0, 0.0]      # Rad/sec
effort: [10.5, 15.2, 8.3, 0.0, 0.0, 0.0]         # Nm (Newton-meters)
```

#### **geometry_msgs/Twist**
Published by motion planners to specify desired robot velocities (linear and angular).

```yaml
# Structure for mobile manipulation (e.g., Spot, HSR)
linear:
  x: 0.5      # Forward velocity (m/s)
  y: 0.0      # Strafe velocity (m/s)
  z: 0.0      # Vertical velocity (m/s)
angular:
  x: 0.0      # Roll rate (rad/s)
  y: 0.0      # Pitch rate (rad/s)
  z: 0.1      # Yaw rate (rad/s)
```

#### **sensor_msgs/Image**
Published by cameras. Contains pixel data, encoding, and metadata.

```yaml
# Structure
header: {seq: 42, stamp: {...}, frame_id: "camera_link"}
height: 480
width: 640
encoding: "rgb8"
is_bigendian: false
step: 1920                    # Bytes per row
data: [255, 128, 64, ...]     # Raw pixel data
```

#### **geometry_msgs/PoseStamped**
Published by perception systems to indicate detected object positions and orientations.

```yaml
# Structure
header: {seq: 1, stamp: {...}, frame_id: "camera_optical_frame"}
pose:
  position:
    x: 0.5      # Position in 3D space (meters)
    y: 0.2
    z: 1.1
  orientation:
    x: 0.0      # Quaternion (x, y, z, w)
    y: 0.0
    z: 0.707
    w: 0.707
```

### Quality of Service (QoS) Policies

ROS 2 allows publishers and subscribers to specify **Quality-of-Service (QoS) policies** that affect message delivery:

#### **Reliability Policy**

- **RELIABLE**: Every message is guaranteed to be delivered (may introduce latency)
- **BEST_EFFORT**: Messages may be lost if the network is congested (lower latency)

**When to use**:
- RELIABLE: Critical messages (grasp commands, emergency stop)
- BEST_EFFORT: High-frequency sensor data (camera frames, IMU readings)

#### **Durability Policy**

- **VOLATILE**: Messages only available to current subscribers
- **TRANSIENT_LOCAL**: New subscribers receive the last message (useful for configuration data)

**Example**:
```python
# Publish robot configuration with TRANSIENT_LOCAL QoS
qos_policy = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
self.config_publisher = self.create_publisher(
    RobotConfig, '/robot/config', qos_profile=qos_policy)
```

#### **History Policy**

- **KEEP_LAST(n)**: Store only the last n messages (default: n=1)
- **KEEP_ALL**: Store all messages until consumed

**Trade-off**: KEEP_ALL uses more memory but ensures no message loss; KEEP_LAST is memory-efficient but may drop old messages if subscribers are slow.

### Topic Naming Conventions

Standardize your topic names for clarity and to avoid conflicts:

```
/robot/<component>/<data_type>

Examples:
/robot/arm/joint_states          # Joint positions/velocities
/robot/gripper/command           # Gripper open/close signal
/robot/camera/rgb/image_raw      # Camera RGB stream
/robot/imu/data                  # IMU acceleration and rotation
/robot/navigation/goal           # Navigation target
/robot/safety/emergency_stop     # Critical safety signal
```

---

## Services and Request-Reply Communication

### What is a Service?

A **service** is a **synchronous, request-reply** mechanism. A client sends a request and waits for a response:

```
Client (e.g., Planning Node)
    ↓ (sends request)
Service Server (e.g., Grasp Planner)
    ↓ (processes request)
    ↑ (sends response)
Client receives response
```

**Key differences from pub-sub**:
- **Synchronous**: Client blocks until the server responds
- **One-to-one**: Each request goes to one specific server
- **Bidirectional**: Information flows both ways

### Service Request-Reply Patterns

#### **Pattern 1: Query Information**
```
Request: "What is the current joint configuration?"
Response: [0.0, 1.57, -1.57, 0.0, 0.0, 0.0]
```

#### **Pattern 2: Perform an Action**
```
Request: "Plan a grasp at position (0.5, 0.2, 1.1) with orientation Q"
Response: { success: true, trajectory: [...], duration: 2.3 }
```

#### **Pattern 3: Enable/Disable Functionality**
```
Request: "Enable collision avoidance"
Response: { success: true, message: "Collision avoidance enabled" }
```

### Service Definition Example

Services are defined in `.srv` files (ROS 2 Interface Definition Language):

```
# GraspPlanning.srv
# Request
geometry_msgs/PoseStamped target_pose
string gripper_type
---
# Response
bool success
trajectory_msgs/JointTrajectory planned_trajectory
float32 execution_time_sec
string error_message
```

A client calls this service like:
```
grasp_client.call_async(GraspPlanning.Request(
    target_pose=pose_stamped,
    gripper_type="parallel_gripper"))
```

### When to Use Services vs. Topics

| Scenario | Use Service | Use Topic | Reason |
|----------|------------|----------|--------|
| Stream camera frames at 30 Hz | ❌ | ✅ | Continuous, asynchronous data; latency-tolerant |
| Publish joint positions every 10 ms | ❌ | ✅ | High-frequency updates; subscribers decide consumption rate |
| Request a grasp plan for an object | ✅ | ❌ | Synchronous request; client needs answer before proceeding |
| Emergency stop signal | ✅ or ✅ | Both work; use service for critical, latency-sensitive commands | |
| Log sensor data for analysis | ❌ | ✅ | Passive consumption; no response needed |
| Check if gripper is gripping | ✅ | ❌ | Query current state; expect immediate response |

**Decision rule**:
- **Use a service if** the caller needs a response before proceeding (request-reply semantics)
- **Use a topic if** the sender doesn't need acknowledgment or if there are multiple subscribers (pub-sub semantics)

---

## Practical Context: Communication in Humanoid Robots

### Example 1: Obstacle Detection Pipeline

A humanoid robot detects obstacles in front of it and adjusts its walking path.

```
Perception Layer:
  Camera Publisher → /robot/camera/rgb/image_raw (topic)
  Obstacle Detector Node (subscriber)
         ↓
    Process image, detect obstacles
         ↓
  Publish Obstacles → /robot/perception/obstacles (topic)

Planning Layer:
  Path Planner Node (subscriber to /robot/perception/obstacles)
         ↓
    Compute collision-free path
         ↓
  Publish Planned Trajectory → /robot/planning/trajectory (topic)

Action Layer:
  Motor Controller Node (subscriber to /robot/planning/trajectory)
         ↓
    Execute joint commands
         ↓
  Publish Joint States → /robot/arm/joint_states (topic)
```

**Why topics?** All data is continuous, asynchronous, and decoupled. The camera doesn't know about obstacles; the planner doesn't know about the camera—they're independent subsystems.

### Example 2: Grasp Command Pipeline

A humanoid reaches for an object and grasps it. The grasping command is synchronous—the arm waits for grasp confirmation.

```
User/AI Agent:
  "Grasp object at (0.5, 0.2, 1.1)" → Service Call
         ↓
Grasp Planning Service Server:
  Inverse kinematics calculation
  Collision checking
  Trajectory generation
         ↓
  Returns: {success: true, trajectory: [...]}
         ↓
Motor Controller subscribes to /robot/arm/planned_trajectory
  Executes joint commands
         ↓
Publishes /robot/arm/joint_states (continuous feedback)
```

**Why a service for grasp planning?** The AI agent needs confirmation that the grasp is feasible before committing to the action. If the service returns `success: false`, the agent can retry with a different pose.

### Example 3: Motor Feedback Loop

A humanoid's arm controller continuously receives joint position feedback.

```
Joint Sensor Publisher:
  Publishes every 10 ms → /robot/arm/joint_states
         ↓ (asynchronous)
  Subscribers:
    - Safety Monitor (checks if joints are in safe range)
    - Motor Controller (reads current state for next command)
    - Data Logger (records for analysis)
         ↓
  No blocking; all subscribers consume at their own pace
```

**Why topics?** Continuous feedback stream with multiple independent consumers.

---

## Communication Patterns in Humanoids

### Pattern 1: Perception → Planning → Action

The classic three-layer architecture for robot control:

```
┌─────────────────────────────────────────────────────────────┐
│                    Humanoid Robot System                     │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  PERCEPTION LAYER:                                           │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ Camera Pub  Depth Pub  IMU Pub   Force/Torque Pub   │   │
│  │     ↓           ↓         ↓            ↓             │   │
│  │ Obstacle Detector, Object Recognizer, Localization  │   │
│  │     ↓                          ↓                     │   │
│  │ Obstacles Topic ─────────→  Object Poses Topic      │   │
│  └──────────────────────────────────────────────────────┘   │
│                                   ↓                          │
│  PLANNING LAYER:                                            │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ Path Planner      Motion Planner     Grasp Planner   │   │
│  │ (service calls)   (service calls)    (service calls) │   │
│  │     ↓                 ↓                  ↓           │   │
│  │ Trajectory Topic ←─ Trajectory Topic ← Trajectory    │   │
│  └──────────────────────────────────────────────────────┘   │
│                                   ↓                          │
│  ACTION LAYER:                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ Motor Controllers (subscribes to Trajectory Topic)   │   │
│  │     ↓                                                │   │
│  │ Publish Joint Commands ─→ Actuators                 │   │
│  │ Publish Joint Feedback ←─ Joint Sensors             │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

**Data flow**:
1. Sensors publish raw data to topics (camera, IMU, force-torque)
2. Perception nodes subscribe, process, and publish interpreted data (obstacles, objects)
3. Planning nodes subscribe to perception data, call services for feasibility checks, and publish trajectories
4. Action nodes subscribe to trajectories and execute commands
5. Feedback loops: Controllers publish joint states; planners subscribe to verify execution

### Pattern 2: Multi-Agent Coordination

Two robots must coordinate actions without collision:

```
Robot 1:
  Motion Planner → /shared/robot_1/trajectory (topic)
                      ↓
              Shared Namespace

Robot 2:
  Motion Planner → /shared/robot_2/trajectory (topic)
                      ↓
              Shared Namespace
                      ↓
  Coordination Service (requests all robot trajectories, checks collisions)
    If collision detected → respond with adjusted trajectories
```

**Pattern**: Each robot publishes its planned trajectory to a shared namespace. A coordination service aggregates all trajectories, checks for collisions, and responds with adjusted plans if conflicts are detected.

### Pattern 3: Hierarchical Control with Mode Switching

Robot operates in different modes (idle, walking, grasping, manipulating):

```
User Input / AI Decision
       ↓
  Mode Selector Service
  ├─ IDLE → Disable all motion controllers
  ├─ WALKING → Activate leg controllers, subscribe to walking target
  ├─ GRASPING → Activate arm controller, subscribe to grasp target
  └─ MANIPULATION → Activate arm + gripper, coordinate movement

Current Mode Topic: /robot/mode
  ↓ (all controllers subscribe)
     Controllers adjust behavior based on mode
```

**Benefit**: Clean separation of concerns. Each controller only responds to its mode, reducing complexity.

---

## Edge Cases and Synchronization

### Challenge 1: Node Failures and Recovery

**Scenario**: A perception node crashes. Planning and action nodes are waiting for data.

```
Perception Node CRASHES ✗
       ↓
Perception Topic (/robot/perception/obstacles) stops receiving data
       ↓
Planner timeout → Planning fails → Action blocked
```

**Solution**:
- **Timeout handling**: Set explicit timeouts on topic subscriptions
- **Fallback behavior**: If no perception data for 500 ms, use last known state
- **Health monitoring**: Launch a dedicated monitoring node that checks heartbeats from critical nodes
- **Graceful degradation**: Continue with conservative assumptions (assume obstacles everywhere)

```python
# Pseudocode for timeout handling
last_obstacle_data = None
last_update_time = time.now()

while True:
    if (time.now() - last_update_time) > 0.5:  # 500 ms timeout
        # No data received; use fallback
        apply_conservative_behavior()
    else:
        # Data available; process normally
        plan_with_current_data()
```

### Challenge 2: Message Delays and Out-of-Order Delivery

**Scenario**: Two camera images arrive out of order due to network delays. Image B (newer) arrives before Image A (older).

```
Time:  0 ms    10 ms    20 ms    30 ms    40 ms
Sent:  Image A ─────→   Image B ─────→
Recv:  ─────────────→   ←─────────────
       (Image B arrives first!)
```

**Solution**:
- **Timestamping**: Every message includes a `header.stamp` (ROS Time)
- **Sorting by timestamp**: Process messages in the order of their sensor timestamp, not arrival order
- **Message synchronization**: Use message_filters to synchronize messages from multiple sensors

```python
# Use ROS 2 message_filters to synchronize camera and depth
from message_filters import ApproximateTimeSynchronizer

sync = ApproximateTimeSynchronizer(
    [camera_sub, depth_sub],
    queue_size=10,
    slop=0.1)  # 100 ms tolerance
sync.registerCallback(on_image_and_depth)

def on_image_and_depth(image_msg, depth_msg):
    # Both messages are approximately synchronized
    process_stereo_pair(image_msg, depth_msg)
```

### Challenge 3: Conflicting Commands (Multi-Agent Coordination)

**Scenario**: Two planning nodes both issue grasp commands to the same gripper.

```
Planner 1: "Open gripper"  ──┐
                              ├─→ Gripper Controller
Planner 2: "Close gripper" ──┘
       ↓ (conflict!)
Which command wins?
```

**Solution**:
- **Single writer principle**: Only one node can write commands to a topic (the "master controller")
- **Service-based arbitration**: Planners call a service; the service enforces mutual exclusion
- **Command queuing**: Queue commands and execute sequentially, or reject conflicting commands

```python
# Service-based arbitration
class GraspArbitrator:
    def handle_grasp_request(self, request):
        if self.gripper_in_use:
            return Response(success=False, message="Gripper busy")
        self.gripper_in_use = True
        self.issue_grasp_command(request)
        self.gripper_in_use = False
        return Response(success=True)
```

### Challenge 4: Latency-Critical Tasks

**Scenario**: Emergency stop signal must reach the motor controller within 50 ms.

```
Safety Monitor detects obstacle
       ↓
Publish /robot/safety/emergency_stop
       ↓ (network delay, queue delay, processing delay)
Motor Controller receives signal (50 ms deadline)
```

**Solution**:
- **High-priority QoS**: Set reliability=RELIABLE and priority=HIGH
- **Dedicated channel**: Use a separate, lower-latency topic for critical signals
- **Redundant communication**: Send emergency stop on both ROS 2 topic and a hardwired interrupt line

```python
# High-priority emergency stop
qos_policy = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST(1),
    deadline=Duration(milliseconds=50))
emergency_pub = self.create_publisher(
    EmergencyStop, '/robot/safety/emergency_stop',
    qos_profile=qos_policy)
```

### Challenge 5: Bandwidth Constraints

**Scenario**: A humanoid is transmitting camera video (30 MB/s) and joint states (1 kB/s) over a 100 Mbps network.

```
Total bandwidth needed: 30 MB/s + overhead > 100 Mbps available
       ↓
Network congestion → messages dropped → control instability
```

**Solution**:
- **Compression**: Compress video with H.264 codec; publish JPEG instead of raw RGB
- **Frequency adaptation**: Lower publication rate for non-critical data
- **Lossy QoS**: Use BEST_EFFORT for high-bandwidth data (camera); RELIABLE for critical data (grasp commands)

```python
# Publish compressed video instead of raw frames
self.compressed_image_pub = self.create_publisher(
    CompressedImage, '/robot/camera/rgb/image_raw/compressed',
    qos_profile=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT))
```

---

## Summary

ROS 2 communication is built on two fundamental patterns:

1. **Topics (Pub-Sub)**: Asynchronous, one-way message distribution. Use for continuous sensor data, decoupled subsystems, and scenarios with multiple subscribers.

2. **Services (Request-Reply)**: Synchronous, bidirectional communication. Use for queries, synchronous operations, and scenarios where the caller needs a response before proceeding.

**Key takeaways**:
- Nodes are independent computational units communicating via topics and services
- Design your communication topology using the **Perception → Planning → Action** pattern
- Use Quality-of-Service policies to ensure reliable, low-latency communication for time-critical tasks
- Handle edge cases (node failures, delays, conflicts) with timeouts, fallback behavior, and arbitration mechanisms
- Balance bandwidth, latency, and reliability based on your humanoid's requirements

**Next chapter**: Learn how to implement these communication patterns in Python using rclpy.

---

## Review Questions

1. **What is the primary difference between a topic and a service in ROS 2?**

2. **Design a communication topology for a humanoid robot that needs to:**
   - **Detect obstacles** using a camera and Lidar
   - **Plan a grasp** for an object
   - **Execute the grasp** with the gripper

   Which nodes would you create? Which would use topics, and which would use services?

3. **Explain when you would use RELIABLE vs. BEST_EFFORT QoS policies. Give two examples for each.**

4. **A humanoid robot publishes joint states at 100 Hz. Three different nodes subscribe to this data:**
   - The motion controller (needs updates every 10 ms)
   - The data logger (records for offline analysis)
   - The safety monitor (checks for anomalies every 100 ms)

   **Why is pub-sub the right choice here, not services?**

5. **What is the node lifecycle in ROS 2, and why is it important for robot control?**

6. **Describe a scenario where an emergency stop signal must be delivered in under 50 ms. How would you ensure this latency guarantee?**

7. **Two planning nodes both try to command the same gripper to open and close simultaneously. What communication pattern would you use to resolve this conflict?**

8. **Explain the role of the `header.stamp` field in ROS 2 messages. Why is it critical for multi-sensor fusion?**

9. **A sensor publishes at 30 Hz, but due to network delays, messages sometimes arrive out of order. How would you ensure your processing code handles this correctly?**

10. **Design a simple Perception → Planning → Action pipeline for a humanoid robot that navigates to and picks up an object. For each component, specify whether it publishes topics, calls services, or both.**

11. **What does "Quality of Service" (QoS) mean in ROS 2? Name three QoS policies and explain their trade-offs.**

12. **A humanoid robot receives a high-bandwidth video stream (30 MB/s) over a 100 Mbps network link. Propose three strategies to stay within bandwidth limits without losing critical functionality.**

13. **Explain why multi-agent robots need a **shared namespace** for their communication topics. Give an example.**

14. **What is the difference between `TRANSIENT_LOCAL` and `VOLATILE` durability in QoS? When would you use each?**

15. **A planning node times out waiting for perception data. Design a fallback strategy that allows the robot to continue operating safely.**

---

## References

- [ROS 2 Humble Documentation: Nodes](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html)
- [ROS 2 Humble Documentation: Topics](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html)
- [ROS 2 Humble Documentation: Services](https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html)
- [ROS 2 Quality of Service Policies](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [Standard ROS 2 Message Types](https://github.com/ros2/common_interfaces)
