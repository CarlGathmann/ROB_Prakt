from os import path
from threading import Thread
import time 
import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from morris_interface.srv import MoveService
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5e

class RobotControl(Node):
    def __init__(self):
        super().__init__('ur5e_control_cal')
        ''' MOVE SERVICE '''
        self.srv = self.create_service(MoveService, 'move',
                                       self.move_executed_callback)  # service for moving/controlling the robot
    def move_executed_callback(self, request, response):
        callback_node = Node('callback_node')

        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=callback_node,
            joint_names=ur5e.joint_names(),
            base_link_name=ur5e.base_link_name(),
            end_effector_name=ur5e.end_effector_name(),
            group_name=ur5e.MOVE_GROUP_ARM,
            callback_group=callback_group,
            ignore_new_calls_while_executing=True
        )

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(callback_node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        self.get_logger().info(f"Goal Pose: {request.position}, {request.rotation}")
        moveit2.move_to_pose(position=request.position, quat_xyzw=request.rotation, cartesian=False, frame_id="base")
        moveit2.wait_until_executed()
        self.get_logger().info(f"GOAL REACHED :)")
        response.done = True
        executor.shutdown()
        return response

def main():
    rclpy.init()

    node = RobotControl()

    rclpy.spin(node)
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
