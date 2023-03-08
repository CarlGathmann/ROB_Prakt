import sys
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from morris_interface.srv import TrackingData
from morris_interface.srv import MoveService

ROBOT_TO_TRACKING = np.array(pd.read_csv('~/floating_ws/end_effector_to_marker.csv'))

class ControlClient(Node):
    def __init__(self):
        super().__init__('control_client')
        self.cli = self.create_client(MoveService, 'move')
        self.req = MoveService.Request()

    def send_request(self, pos, rot, cartesian):
        self.req.position = pos
        self.req.rotation = rot
        self.req.cartesian = cartesian
        self.cli.wait_for_service()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

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

def get_board_from_tracking():
    tracking = BoardClient()
    pos_board, rot_board = tracking.send_request('board')
    tracking.get_logger().info('Request sent')
    rot_board = Rot.from_quat(rot_board).as_matrix()
    tracking_to_board = np.zeros((4, 4))
    tracking_to_board[:3, :3] = rot_board
    tracking_to_board[:3, -1] = pos_board
    tracking_to_board[3] = [0, 0, 0, 1]
    return tracking_to_board


def get_robot_to_board():
    tracking_to_board = get_board_from_tracking()
    print(tracking_to_board)
    robot_to_board = ROBOT_TO_TRACKING @ tracking_to_board
    print(robot_to_board)
    return robot_to_board

def main():
    rclpy.init()
    minimal_client = ControlClient()
    pos = get_robot_to_board()[:3, -1]
    pos = [pos[0], pos[1], pos[2]]
    response = minimal_client.send_request(pos, [0.0, 1.0, 0.0, 0.0], True)
    minimal_client.get_logger().info('Request sent')
    self.get_logger().info(f"pos{pos}")
    minimal_client.get_logger().info('finished')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
