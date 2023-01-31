#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from kinova_msgs.action import ArmJointAngles

class FingersPoseClient(Node):
    def __init__(self):
        super().__init__('cartesian_pose_client')

        self.position = [0.,0.,0.,0.,0.,0.,0.]

        self.client = ActionClient(self, ArmJointAngles, '/j2n6s300_driver/joint_angles')

        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

    def send_goal(self):
        goal = ArmJointAngles.Goal()
        goal.angles.joint1 = self.position[0]
        goal.angles.joint2 = self.position[1]
        goal.angles.joint3 = self.position[2]
        goal.angles.joint4 = self.position[3]
        goal.angles.joint5 = self.position[4]
        goal.angles.joint6 = self.position[5]

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

    node = FingersPoseClient()
    node.send_goal()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
