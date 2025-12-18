#!/usr/bin/env python3
"""
Simple ROS 2 Subscriber Example
Subscribes to /robot/joint_states topic and prints received messages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):
    """A simple node that subscribes to joint state messages."""

    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Create a subscription to JointState messages
        self.subscription = self.create_subscription(
            JointState,
            '/robot/joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Joint State Subscriber started')

    def listener_callback(self, msg: JointState):
        """Callback function called when a message is received."""
        self.get_logger().info(
            f'Received joint state for {len(msg.name)} joints:\n'
            f'  Joints: {msg.name}\n'
            f'  Positions: {msg.position}\n'
            f'  Velocities: {msg.velocity}'
        )


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    subscriber = JointStateSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
