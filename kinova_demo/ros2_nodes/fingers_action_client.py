#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from kinova_msgs.action import SetFingersPosition

class FingersPoseClient(Node):
    def __init__(self):
        super().__init__('cartesian_pose_client')

        # self.position = [50.,50.,50.] # Close
        self.position = [0.,0.,0.] # Open

        self.client = ActionClient(self, SetFingersPosition, '/j2n6s300_driver/finger_positions')

        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

    def send_goal(self):
        goal = SetFingersPosition.Goal()
        goal.fingers.finger1 = self.position[0]
        goal.fingers.finger2 = self.position[1]
        goal.fingers.finger3 = self.position[2]

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
