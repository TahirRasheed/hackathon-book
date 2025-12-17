#!/usr/bin/env python3
"""
Simple ROS 2 Service Client Example
Calls a hypothetical /grasp_object service to request grasp execution.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class GraspClient(Node):
    """A simple service client node."""

    def __init__(self):
        super().__init__('grasp_client')

        # For demonstration, we won't actually create the service client
        # since the /grasp_object service won't exist in this example.
        # In a real system, you would:
        self.get_logger().info('Grasp Client initialized')

    def call_grasp_service(self, target_pose):
        """Call the grasp service (demonstration)."""
        self.get_logger().info(
            f'Would call /grasp_object service with target pose:\n'
            f'  Position: x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}\n'
            f'  Orientation: x={target_pose.orientation.x}, y={target_pose.orientation.y}, '
            f'z={target_pose.orientation.z}, w={target_pose.orientation.w}'
        )

        # In a real ROS 2 system, you would:
        # 1. Create a service client:
        #    self.cli = self.create_client(Grasp, '/grasp_object')
        # 2. Wait for the service:
        #    while not self.cli.wait_for_service(timeout_sec=1.0):
        #        self.get_logger().info('Service not available, waiting again...')
        # 3. Send the request:
        #    req = Grasp.Request()
        #    req.target_pose = target_pose
        #    future = self.cli.call_async(req)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    client = GraspClient()

    # Create a dummy target pose for demonstration
    target = Pose()
    target.position.x, target.position.y, target.position.z = 0.5, 0.3, 0.8
    target.orientation.w = 1.0  # identity quaternion

    # Call the service (demonstration)
    client.call_grasp_service(target)

    # Shutdown
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
