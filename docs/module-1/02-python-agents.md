# Chapter 3: Python AI Agents, rclpy, and Humanoid Descriptions (URDF)

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Write functional ROS 2 nodes in Python** using rclpy, including publishers, subscribers, and service clients
2. **Design and execute publisher-subscriber patterns** for continuous sensor data and robot state
3. **Implement synchronous service calls** for query and action requests
4. **Understand URDF (Unified Robot Description Format)** XML structure and robot kinematic chains
5. **Parse URDF files programmatically** to extract link hierarchies, joint limits, and inertia properties
6. **Integrate Python AI planning logic with ROS 2 communication** to create autonomous humanoid agents
7. **Handle real-world challenges** in ROS 2 Python programming (timeouts, exceptions, resource cleanup)

---

## Core Concepts: Introduction to rclpy

### What is rclpy?

**rclpy** (ROS 2 Client Library for Python) is the official Python interface to ROS 2. It enables Python developers to:
- Create nodes that participate in the ROS 2 graph
- Publish and subscribe to topics
- Call and provide services
- Define and execute actions
- Access ROS 2 parameters and timers

rclpy abstracts the underlying ROS 2 middleware (DDS) and provides a Pythonic API.

### Installation and Setup

rclpy is automatically installed with a standard ROS 2 installation:

```bash
# Verify rclpy is available
python3 -c "import rclpy; print(rclpy.__version__)"

# Source your ROS 2 environment
source /opt/ros/humble/setup.bash
```

### Node Lifecycle in rclpy

Every rclpy node follows this pattern:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize publishers, subscribers, timers here

    def some_method(self):
        # Your logic here
        pass

def main(args=None):
    rclpy.init(args=args)                    # Initialize ROS 2
    node = MyNode()                          # Create node
    rclpy.spin(node)                         # Event loop: listen for messages
    node.destroy_node()                      # Cleanup
    rclpy.shutdown()                         # Shutdown ROS 2

if __name__ == '__main__':
    main()
```

---

## Writing Publishers in Python

### Publisher Basics

A **publisher** sends messages to a topic. Other nodes can subscribe and receive copies of those messages.

### Example: Simple Joint State Publisher

Let's create a publisher that broadcasts joint state information for a 6-DOF robotic arm:

```python
#!/usr/bin/env python3
"""
ROS 2 Publisher Example: Joint State Publisher
Continuously publishes joint state information (positions, velocities, efforts).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStatePublisher(Node):
    """Node that publishes joint state information."""

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create a publisher for joint states
        # Arguments:
        #   1. Message type: JointState
        #   2. Topic name: '/robot/joint_states'
        #   3. Queue size: 10 (buffer up to 10 messages)
        self.publisher_ = self.create_publisher(
            JointState,
            '/robot/joint_states',
            qos_profile=10)

        # Timer: call callback every 0.1 seconds (10 Hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_joint_states)

        self.simulation_time = 0.0
        self.get_logger().info('Joint State Publisher started')

    def publish_joint_states(self):
        """Callback: Publish joint state every 0.1 seconds."""

        # Create a JointState message
        msg = JointState()

        # Set header (timestamp and frame reference)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'robot_base'

        # Define joint names
        msg.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_flex_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Simulate joint positions with sinusoidal motion
        msg.position = [
            math.sin(self.simulation_time) * 1.57,
            0.785 + 0.5 * math.cos(self.simulation_time),
            -1.57 + 0.3 * math.sin(self.simulation_time),
            math.cos(self.simulation_time) * 0.5,
            0.0,
            0.0
        ]

        # Calculate velocities (derivative of position)
        msg.velocity = [
            math.cos(self.simulation_time) * 1.57,
            -0.5 * math.sin(self.simulation_time),
            0.3 * math.cos(self.simulation_time),
            -math.sin(self.simulation_time) * 0.5,
            0.0,
            0.0
        ]

        # Simulate joint efforts (torques)
        msg.effort = [50.0, 75.0, 30.0, 5.0, 5.0, 5.0]

        # Publish the message
        self.publisher_.publish(msg)

        # Log periodically
        if int(self.simulation_time * 10) % 10 == 0:
            self.get_logger().info(
                f'Published {len(msg.name)} joint states')

        self.simulation_time += 0.1


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key concepts**:
- `create_publisher()` registers the publisher on the topic
- `create_timer()` calls a callback function at regular intervals
- `get_clock().now().to_msg()` gets the current timestamp in ROS time
- `publisher_.publish(msg)` sends the message

**To run this example**:
```bash
ros2 run <package> 02-python-agents.py
```

---

## Writing Subscribers in Python

### Subscriber Basics

A **subscriber** receives messages from a topic. When a message arrives, a callback function is invoked.

### Example: Simple Joint State Subscriber

```python
#!/usr/bin/env python3
"""
ROS 2 Subscriber Example
Subscribes to /robot/joint_states and processes messages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):
    """Node that subscribes to joint state messages."""

    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Create a subscription
        # Arguments:
        #   1. Message type: JointState
        #   2. Topic name: '/robot/joint_states'
        #   3. Callback function: called when message arrives
        #   4. Queue size: 10
        self.subscription = self.create_subscription(
            JointState,
            '/robot/joint_states',
            self.listener_callback,
            10
        )

        self.get_logger().info('Joint State Subscriber started')

    def listener_callback(self, msg: JointState):
        """Callback: Process received joint state message."""

        self.get_logger().info(
            f'Received joint state for {len(msg.name)} joints:\n'
            f'  Joints: {msg.name}\n'
            f'  Positions (rad): {[round(p, 3) for p in msg.position]}\n'
            f'  Velocities (rad/s): {[round(v, 3) for v in msg.velocity]}'
        )

        # Example: Detect abnormal joint velocities
        for name, velocity in zip(msg.name, msg.velocity):
            if abs(velocity) > 2.0:  # Warning threshold
                self.get_logger().warn(
                    f'Joint {name} has high velocity: {velocity} rad/s')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = JointStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key concepts**:
- `create_subscription()` registers the subscriber on the topic
- The callback function is called **asynchronously** when messages arrive
- `rclpy.spin()` blocks and processes messages until interrupted
- Subscribers can process messages at their own pace; messages are queued

**To run**:
```bash
# In one terminal (publisher)
ros2 run <package> 02-python-agents.py

# In another terminal (subscriber)
ros2 run <package> 02-python-agents.py
```

---

## Calling Services from Python

### Service Client Basics

A **service client** sends a request to a service and waits for a response. Unlike pub-sub, this is synchronous.

### Example: Grasp Planning Service Client

```python
#!/usr/bin/env python3
"""
ROS 2 Service Client Example
Calls a grasp planning service to request a grasp trajectory.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class GraspClient(Node):
    """Node that calls the grasp planning service."""

    def __init__(self):
        super().__init__('grasp_client')
        self.get_logger().info('Grasp Client initialized')

    def call_grasp_service(self, target_pose):
        """
        Call the grasp planning service.

        In a real system, the service interface might be:

        geometry_msgs/PoseStamped target_pose
        string gripper_type
        ---
        bool success
        trajectory_msgs/JointTrajectory trajectory
        float32 execution_time_sec
        """

        self.get_logger().info(
            f'Requesting grasp at position: '
            f'({target_pose.position.x:.2f}, '
            f'{target_pose.position.y:.2f}, '
            f'{target_pose.position.z:.2f})'
        )

        # In a real implementation:
        # 1. Create a service client:
        #    self.cli = self.create_client(GraspPlan, '/grasp_object')
        #
        # 2. Wait for service availability:
        #    while not self.cli.wait_for_service(timeout_sec=1.0):
        #        self.get_logger().info('Service not available, waiting...')
        #
        # 3. Create request and send:
        #    req = GraspPlan.Request()
        #    req.target_pose = target_pose
        #    req.gripper_type = 'parallel_gripper'
        #    future = self.cli.call_async(req)
        #    rclpy.spin_until_future_complete(self, future)
        #    response = future.result()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = GraspClient()

    # Create a target pose (example)
    target = Pose()
    target.position.x = 0.5
    target.position.y = 0.3
    target.position.z = 0.8
    target.orientation.w = 1.0  # identity quaternion

    node.call_grasp_service(target)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key concepts**:
- `create_client()` creates a service client
- `wait_for_service()` blocks until the service is available
- `call_async()` sends the request asynchronously
- `rclpy.spin_until_future_complete()` waits for the response

---

## Understanding URDF: Unified Robot Description Format

### What is URDF?

**URDF** is an XML format that describes a robot's structure:
- **Links**: Rigid bodies with mass and geometry
- **Joints**: Connections between links with motion constraints
- **Inertia**: Mass distribution properties for dynamics simulation

### URDF Structure: An Example

Here's a simplified URDF for a humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base link (torso) -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head link -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting torso to head -->
  <joint name="head_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <axis xyz="0 0 1"/>
    <!-- Joint angle limits: -90° to +90° (in radians) -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left shoulder -->
  <link name="left_shoulder">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>

  <!-- ... more links and joints ... -->

</robot>
```

### Key URDF Elements

| Element | Purpose | Example |
|---------|---------|---------|
| `<robot>` | Root element; names the robot | `<robot name="humanoid">` |
| `<link>` | Defines a rigid body | `<link name="left_arm">` |
| `<joint>` | Connects two links | `<joint name="elbow" type="revolute">` |
| `<parent link="...">` | Parent link of the joint | Links robot to base_link |
| `<child link="...">` | Child link (moved by joint) | link="hand" |
| `<axis xyz="...">` | Rotation axis for revolute joints | xyz="1 0 0" (x-axis) |
| `<limit>` | Joint motion constraints | lower, upper, effort, velocity |
| `<inertial>` | Mass and inertia properties | For dynamics simulation |
| `<mass>` | Link mass in kg | value="2.5" |
| `<inertia>` | Inertia tensor (moment of inertia) | 6 values: ixx, ixy, ixz, iyy, iyz, izz |

### Joint Types

- **revolute**: Rotational joint with limited range (e.g., elbow)
- **continuous**: Unlimited rotation (e.g., wheel)
- **prismatic**: Linear sliding motion (e.g., door on track)
- **fixed**: No motion; locks two links together

---

## Parsing URDF Files in Python

### URDF Parser Example

```python
#!/usr/bin/env python3
"""
Simple URDF Parser: Extract robot structure from XML.
"""

import xml.etree.ElementTree as ET


class URDFParser:
    """Parse and analyze URDF files."""

    def __init__(self, urdf_file_path):
        """Initialize parser with URDF file path."""
        self.urdf_file = urdf_file_path
        self.tree = ET.parse(urdf_file_path)
        self.root = self.tree.getroot()
        self.robot_name = self.root.get('name', 'unknown')

    def get_links(self):
        """Extract all links from the URDF."""
        links = []
        for link in self.root.findall('link'):
            link_name = link.get('name')
            links.append(link_name)
        return links

    def get_joints(self):
        """Extract all joints and their properties."""
        joints = []
        for joint in self.root.findall('joint'):
            joint_name = joint.get('name')
            joint_type = joint.get('type')
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')

            # Extract joint limits
            limit = joint.find('limit')
            lower = float(limit.get('lower', 0)) if limit is not None else 0
            upper = float(limit.get('upper', 0)) if limit is not None else 0

            joints.append({
                'name': joint_name,
                'type': joint_type,
                'parent': parent,
                'child': child,
                'lower_limit': lower,
                'upper_limit': upper
            })
        return joints

    def print_structure(self):
        """Print the robot structure."""
        print(f"Robot: {self.robot_name}\n")

        links = self.get_links()
        print(f"Links ({len(links)}):")
        for link in links:
            print(f"  - {link}")

        joints = self.get_joints()
        print(f"\nJoints ({len(joints)}):")
        for joint in joints:
            print(f"  - {joint['name']} ({joint['type']})")
            print(f"    {joint['parent']} → {joint['child']}")
            print(f"    Range: [{joint['lower_limit']:.2f}, {joint['upper_limit']:.2f}] rad")


def main():
    """Main function."""
    urdf_file = 'docs/module-1/code-examples/02-humanoid-example.urdf'

    try:
        parser = URDFParser(urdf_file)
        parser.print_structure()
    except FileNotFoundError:
        print(f"Error: URDF file '{urdf_file}' not found.")
    except ET.ParseError as e:
        print(f"Error parsing URDF: {e}")


if __name__ == '__main__':
    main()
```

**Output** (for the humanoid example):
```
Robot: simple_humanoid

Links (13):
  - base_link
  - head
  - left_shoulder
  - left_elbow
  - left_hand
  - right_shoulder
  - right_elbow
  - right_hand
  - left_hip
  - left_knee
  - left_foot
  - right_hip
  - right_knee
  - right_foot

Joints (12):
  - head_joint (revolute)
    base_link → head
    Range: [-1.57, 1.57] rad
  - left_shoulder_pitch (revolute)
    base_link → left_shoulder
    Range: [-1.57, 1.57] rad
  ... (more joints)
```

---

## Building AI Agents for Humanoid Control

### Architecture: Integrating URDF, ROS 2 Communication, and AI Planning

A complete AI agent for humanoid control integrates three components:

```
┌──────────────────────────────────────────────────────────────┐
│                    AI Planning Agent Node                     │
├──────────────────────────────────────────────────────────────┤
│                                                                │
│  1. Robot Model (URDF):                                      │
│     ├─ Load URDF file                                        │
│     ├─ Extract link hierarchy and joint limits              │
│     └─ Build kinematic chain for IK calculations             │
│                                                                │
│  2. Perception Subscriber:                                   │
│     ├─ Subscribe to /robot/perception/obstacles             │
│     ├─ Subscribe to /robot/camera/rgb/image_raw            │
│     └─ Build world model (object positions, obstacles)       │
│                                                                │
│  3. Planning Logic:                                          │
│     ├─ Inverse kinematics (URDF → joint angles)             │
│     ├─ Collision checking (URDF + world model)             │
│     └─ Trajectory generation                                │
│                                                                │
│  4. Action Publisher:                                        │
│     └─ Publish /robot/planning/trajectory                   │
│                                                                │
│  5. Service Clients:                                         │
│     ├─ Call /grasp_planner (if object is graspable)         │
│     └─ Call /safety_check (verify trajectory)               │
│                                                                │
└──────────────────────────────────────────────────────────────┘
```

### Example: Simple Planning Agent Pseudocode

```python
#!/usr/bin/env python3
"""
Example: AI Planning Agent for Humanoid Robot.

This agent:
1. Reads robot description (URDF)
2. Subscribes to perception (obstacles, objects)
3. Plans grasp trajectory using kinematics
4. Publishes planned trajectory
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
import xml.etree.ElementTree as ET


class PlanningAgent(Node):
    """AI agent for humanoid manipulation tasks."""

    def __init__(self, urdf_file):
        super().__init__('planning_agent')

        # Load robot model from URDF
        self.robot_urdf = URDFParser(urdf_file)
        self.get_logger().info(
            f'Loaded robot: {self.robot_urdf.robot_name} '
            f'with {len(self.robot_urdf.get_joints())} joints')

        # Subscribe to perception data
        self.object_subscriber = self.create_subscription(
            PoseStamped,
            '/robot/perception/detected_objects',
            self.on_object_detected,
            10
        )

        # Subscribe to joint state feedback
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/robot/arm/joint_states',
            self.on_joint_state,
            10
        )

        # Publisher for planned trajectory
        self.trajectory_publisher = self.create_publisher(
            Twist,  # In real system: trajectory_msgs/JointTrajectory
            '/robot/planning/trajectory',
            10
        )

        # State tracking
        self.current_joint_state = None
        self.detected_objects = []

    def on_object_detected(self, msg: PoseStamped):
        """Callback: New object detected by perception system."""
        self.get_logger().info(
            f'Detected object at ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, '
            f'{msg.pose.position.z:.2f})')

        self.detected_objects.append(msg)

        # Trigger planning
        self.plan_grasp(msg.pose)

    def on_joint_state(self, msg: JointState):
        """Callback: Update current joint state."""
        self.current_joint_state = msg

    def plan_grasp(self, target_pose):
        """
        Plan a grasp trajectory to the target pose.

        Steps:
        1. Inverse kinematics: target_pose → joint angles
        2. Collision checking: verify trajectory doesn't hit obstacles
        3. Generate trajectory: smooth path from current to target
        4. Publish trajectory
        """

        if self.current_joint_state is None:
            self.get_logger().warn('No joint state available yet')
            return

        # Step 1: Inverse Kinematics
        target_joint_angles = self.inverse_kinematics(target_pose)
        if target_joint_angles is None:
            self.get_logger().error('IK solution not found')
            return

        # Step 2: Collision Check
        if not self.check_collision_free(target_joint_angles):
            self.get_logger().warn('Collision detected in planned trajectory')
            return

        # Step 3: Trajectory Generation
        trajectory = self.generate_trajectory(
            self.current_joint_state.position,
            target_joint_angles)

        # Step 4: Publish Trajectory
        msg = Twist()  # In real system: JointTrajectory message
        self.trajectory_publisher.publish(msg)
        self.get_logger().info('Trajectory published')

    def inverse_kinematics(self, target_pose):
        """
        Solve inverse kinematics: target end-effector pose → joint angles.

        In a real implementation, you would use:
        - KDL (Kinematics and Dynamics Library)
        - PyBullet
        - Drake (from Toyota Research Institute)
        """
        # Placeholder: return dummy joint angles
        joints = self.robot_urdf.get_joints()
        return [0.0] * len(joints)

    def check_collision_free(self, joint_angles):
        """
        Verify trajectory is collision-free.

        Uses URDF link geometry to check for self-collisions
        and collisions with perceived obstacles.
        """
        # Placeholder: always return True
        return True

    def generate_trajectory(self, current_angles, target_angles):
        """Generate a smooth trajectory using linear interpolation."""
        # Placeholder: return straight-line trajectory
        steps = 10
        trajectory = []
        for i in range(steps):
            t = i / (steps - 1)
            angles = [
                current + t * (target - current)
                for current, target in zip(current_angles, target_angles)
            ]
            trajectory.append(angles)
        return trajectory


class URDFParser:
    """Simplified URDF parser."""
    def __init__(self, urdf_file):
        self.tree = ET.parse(urdf_file)
        self.root = self.tree.getroot()
        self.robot_name = self.root.get('name')

    def get_joints(self):
        return self.root.findall('joint')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    urdf_file = 'docs/module-1/code-examples/02-humanoid-example.urdf'
    agent = PlanningAgent(urdf_file)

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key insights**:
- The agent combines **URDF knowledge** (kinematic chain), **perception** (detected objects), **planning** (IK, collision checking), and **ROS 2 communication** (pub-sub, services)
- This is a simplified example; real systems use libraries like KDL or PyBullet for kinematics
- Error handling and timeouts are critical for robust autonomous systems

---

## Summary

This chapter taught you how to write Python ROS 2 nodes and integrate them with robot descriptions:

**rclpy Patterns**:
- Publishers broadcast continuous data to topics
- Subscribers receive messages asynchronously
- Service clients send requests and wait for responses

**URDF and Robot Models**:
- URDF files describe robot structure (links, joints, inertia)
- Parse URDF with XML to extract kinematic chains
- Use URDF in planning algorithms (IK, collision checking)

**AI Agents for Humanoid Control**:
- Combine perception (subscribers), planning (logic), and action (publishers)
- Use URDF for kinematic understanding
- Handle failures gracefully with timeouts and error checking

**Next steps**: Deploy your agents on a real robot or simulator (Gazebo, MuJoCo) and test autonomous behavior.

---

## Review Questions

1. **What are the three main components of an rclpy node, and how do they interact?**

2. **Explain the difference between a publisher that runs at 100 Hz and a subscriber that processes messages. How are they decoupled?**

3. **Write pseudocode to create a ROS 2 publisher that publishes camera images at 30 Hz. What message type would you use?**

4. **What is the advantage of using service clients instead of topics for requesting a grasp plan?**

5. **Describe the structure of a URDF file. What are the four most important elements?**

6. **A humanoid robot has 15 joints. How would you extract all joint names and their rotation axes from a URDF file using XML parsing?**

7. **In the planning agent example, why is it important to check `self.current_joint_state` before planning?**

8. **What does "inverse kinematics" mean? Why is it needed to grasp an object at a target position?**

9. **Design an AI agent that:**
   - Subscribes to perceived obstacles
   - Plans a collision-free path
   - Publishes the planned trajectory

   Describe the ROS 2 topics, message types, and node structure.

10. **A subscriber receives joint states at 100 Hz, but your collision checking algorithm runs at only 10 Hz. How would you handle this rate mismatch?**

11. **Explain how URDF joint limits (e.g., `<limit lower="-1.57" upper="1.57"/>`) constrain an IK solver.**

12. **What errors might occur when calling a service? How would you handle timeouts in rclpy?**

13. **A humanoid arm has 6 DOF (degrees of freedom). Describe how you would use URDF to verify that a target grasp pose is kinematically feasible.**

14. **What is the role of the header timestamp in JointState messages? Why is it important for multi-agent robots?**

15. **Design a safety-aware planning agent that checks for collisions before publishing a trajectory. What safeguards would you implement?**

---

## Code Examples Location

All code examples referenced in this chapter are available in:
- `01-publisher.py` — Simple joint state publisher
- `01-subscriber.py` — Simple joint state subscriber
- `01-service-client.py` — Grasp service client
- `02-humanoid-example.urdf` — Sample 12-joint humanoid robot
- `02-urdf-parser.py` — URDF parsing utility

To run these examples, ensure your ROS 2 environment is sourced and the package is properly set up.

---

## References

- [rclpy Documentation](https://docs.ros.org/en/humble/Packages/Development-tools/rclpy/index.html)
- [ROS 2 Python Client Library Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [URDF XML Specification](https://wiki.ros.org/urdf/XML)
- [KDL: Kinematics and Dynamics Library](https://github.com/orocos/orocos_kinematics_dynamics)
- [ROS 2 Message Types](https://github.com/ros2/common_interfaces)
