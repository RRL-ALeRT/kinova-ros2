#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from kinova_msgs.action import ArmPose
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion

class CartesianPoseClient(Node):
    def __init__(self):
        super().__init__('cartesian_pose_client')

        self.position = [0.212322831154, -0.257197618484, 0.509646713734]
        self.orientation = [1.63771402836, 1.11316478252, 0.134094119072, 0.]

        self.client = ActionClient(self, ArmPose, '/j2n6s300_driver/tool_pose')

        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

    def send_goal(self):
        goal = ArmPose.Goal()
        goal.pose.header = Header(frame_id='j2n6s300_link_base')
        goal.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
        goal.pose.pose.orientation = Quaternion(x=self.orientation[0], y=self.orientation[1], z=self.orientation[2], w=self.orientation[3])

        self.get_logger().info('Sending goal...')
        self.client.send_goal(goal)

        self.get_logger().info('Waiting for result...')
        self.client.wait_for_result()

        result = self.client.get_result()
        if not result:
            self.get_logger().info('Goal failed')
        else:
            self.get_logger().info(f'Result: {result.pose.pose}')

def main(args=None):
    rclpy.init(args=args)

    node = CartesianPoseClient()
    node.send_goal()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
