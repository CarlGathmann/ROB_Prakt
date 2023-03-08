from os import path
from threading import Thread
import time
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5e
from morris_interface.srv import TrackingData


GRIPPER_STL = "/home/prakt5/floating_ws/src/set_workspace/set_workspace/assests/Gripper.stl"
WALL_STL = "/home/prakt5/floating_ws/src/set_workspace/set_workspace/assests/wall.stl"
TABLE_STL = "/home/prakt5/floating_ws/src/set_workspace/set_workspace/assests/table.stl"
BOARD_STL = "/home/prakt5/floating_ws/src/set_workspace/set_workspace/assests/chip_holder.stl"
ROBOT_TO_TRACKING = np.array(pd.read_csv('~/floating_ws/robot_to_tracking_system.csv'))

print(ROBOT_TO_TRACKING)
class BoardClient(Node):
    def __init__(self):
        super().__init__('tracking_board')
        self.cli = self.create_client(TrackingData, 'board_pose')
        self.req = TrackingData.Request()
    ''' 
    pose needs to be string
    either gripper for the gripper pose or 
    board for the board pose
    '''
    def send_request(self, name):
        self.req.name = name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().position, self.future.result().rotation

class WorkspaceSetter(Node):

    def __init__(self):
        super().__init__('workspace_setter')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter("filepath_gripper", GRIPPER_STL)
        self.declare_parameter("filepath_wall", WALL_STL)
        self.declare_parameter("filepath_table", TABLE_STL)
        self.declare_parameter("filepath_board", BOARD_STL)
        self.client = BoardClient()

    def timer_callback(self):
        callback_group = ReentrantCallbackGroup()
        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=self,
            joint_names=ur5e.joint_names(),
            base_link_name=ur5e.base_link_name(),
            end_effector_name=ur5e.end_effector_name(),
            group_name=ur5e.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Get parameters
        filepath_gripper = self.get_parameter("filepath_gripper").get_parameter_value().string_value
        filepath_wall = self.get_parameter("filepath_wall").get_parameter_value().string_value
        filepath_table = self.get_parameter("filepath_table").get_parameter_value().string_value
        filepath_board = self.get_parameter("filepath_board").get_parameter_value().string_value

        # Determine ID of the collision mesh
        mesh_id_gripper = path.basename(filepath_gripper).split(".")[0]
        mesh_id_wall = path.basename(filepath_wall).split(".")[0]
        mesh_id_table = path.basename(filepath_table).split(".")[0]
        mesh_id_board = path.basename(filepath_board).split(".")[0]
        '''
        self.get_logger().info(f"refresh workspace")
        moveit2.remove_collision_mesh(id=mesh_id_gripper)
        moveit2.remove_collision_mesh(id=mesh_id_board)
        moveit2.remove_collision_mesh(id=mesh_id_table)
        moveit2.remove_collision_mesh(id=mesh_id_wall)
        '''
        # Add wall mesh
        moveit2.add_collision_mesh(
            filepath=filepath_wall, id=mesh_id_wall, position=[-0.6, 0.0, 0.0],
            quat_xyzw=[0.0, 0.0, 0.0, 0.0],
            frame_id="base")
        self.get_logger().info(f"wall added")
        # Add table mesh
        moveit2.add_collision_mesh(
            filepath=filepath_table, id=mesh_id_table, position=[0.0, 0.0, -1.1],
            quat_xyzw=[0.0, 0.0, 0.0, 0.0],
            frame_id="base")
        self.get_logger().info(f"table added")



        # Add board mesh
        # boards position needs to be in robot coordinates. Multiply with tracking to robot matrix.
        rot_matrix = Rot.from_quat(self.client.send_request('board')[1]).as_matrix()
        board_pos = self.client.send_request('board')[0]
        board_matrix = np.zeros((4, 4))
        board_matrix[:3, :3] = rot_matrix
        board_matrix[:3, -1] = board_pos
        board_matrix[3] = [0, 0, 0, 1]
        robot_board_matrix = ROBOT_TO_TRACKING @ board_matrix
        board_in_rob_coord = robot_board_matrix @ board_matrix[:, 1]
        self.get_logger().info(f"Board_pos: {board_in_rob_coord}")
        board_pos = board_in_rob_coord
        board_rot = Rot.from_matrix(robot_board_matrix[:3, :3]).as_quat()

        moveit2.add_collision_mesh(
            filepath=filepath_table, id=mesh_id_board, position=board_pos,
            quat_xyzw=board_rot,
            frame_id="base")
        self.get_logger().info(f"board added")

        '''
        moveit2.add_collision_mesh(
            filepath=filepath_gripper, id=mesh_id_gripper, position=[0.0, 0.0, 0.12],
            quat_xyzw=[0.0, 0.0, 1.0, 0.0],
            frame_id="tool0")
        self.get_logger().info(f"gripper added")
        '''


def main():
    rclpy.init()
    # Create node for this example
    client = BoardClient()
    node = WorkspaceSetter()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
