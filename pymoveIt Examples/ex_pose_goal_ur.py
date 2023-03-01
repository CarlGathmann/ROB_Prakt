#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal_ur.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
import matplotlib.pyplot as plt
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5e

import math
import numpy as np

from functools import reduce
from gettext import translation
import math
import numpy as np

# a script for multiple rotaitinal matrices 

# matrix for a rotation around the x-axis. Matrix size is 4x4
def rotX(deg):
    rad = np.deg2rad(deg)
    return np.array([[1, 0, 0, 0], [0, math.cos(rad), -math.sin(rad), 0], [0 , math.sin(rad), math.cos(rad), 0], [0, 0, 0, 1]])
# matrix for a rotation around the y-axis. Matrix size is 4x4
def rotY(deg):
    rad = np.deg2rad(deg)
    return np.array([[math.cos(rad), 0 , math.sin(rad), 0], [0, 1, 0, 0], [-math.sin(rad), 0, math.cos(rad), 0], [0, 0, 0, 1]])
# matrix for a rotation around the z-axis. Matrix size is 4x4
def rotZ(deg):
    rad = np.deg2rad(deg)
    return np.array([[math.cos(rad), -math.sin(rad), 0, 0], [math.sin(rad), math.cos(rad), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
#matrix for translation
def trans(x, y, z):
    return np.array([[1, 0, 0, x],[0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

rotations = 1
start_pos = np.array([0.1, 0, 0, 1])
cache = []


def spiral(pos):
    m = reduce(np.matmul, [rotZ(5), trans(14,0,0.01)])
    pos = np.transpose(pos)
    return np.matmul(m, pos)

def main():

    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal_ur")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.5, 0.0, 0.25])
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian", True)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()	

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5e.joint_names(),
        base_link_name=ur5e.base_link_name(),
        end_effector_name=ur5e.end_effector_name(),
        group_name=ur5e.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    position = node.get_parameter("position").get_parameter_value().double_array_value


    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()
    
    plt.plot(cache)

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
