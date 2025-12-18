#!/usr/bin/env python3
"""
Simple ROS 2 Publisher Example
Publishes dummy joint state messages on the /robot/joint_states topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


class JointStatePublisher(Node):
    """A simple node that publishes joint states."""

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create a publisher for JointState messages
        self.publisher_ = self.create_publisher(JointState, '/robot/joint_states', 10)

        # Create a timer to publish at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for example data
        self.counter = 0.0
        self.get_logger().info('Joint State Publisher started')

    def timer_callback(self):
        """Callback function called by the timer."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Define joint names (example humanoid joints)
        msg.name = ['left_shoulder_pitch', 'right_shoulder_pitch', 'left_elbow', 'right_elbow']

        # Publish dummy position data (sine wave)
        import math
        self.counter += 0.1
        msg.position = [
            math.sin(self.counter),
            math.sin(self.counter + math.pi),
            math.cos(self.counter),
            math.cos(self.counter + math.pi)
        ]

        # Publish dummy velocity data (all zeros for this example)
        msg.velocity = [0.0, 0.0, 0.0, 0.0]

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing joint positions: {msg.position}')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    publisher = JointStatePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
