import numpy as np
from scipy.spatial.transform import Rotation as Rot
import pandas as pd
import rclpy
from rclpy.node import Node

from morris_interface.srv import MoveService
from morris_interface.srv import TrackingData
from morris_interface.srv import SetUrIo

# TODO: measure values
SAFE_POS = [0.0, 0.0, 0.05, 1.0]
MIDDLE_SHELL = [0.037, 0.037, 0.05, 1]
INNER_SHELL = [0.0723, 0.0723, 0.05, 1]
OFFSET_OUTER_SHELL = 0.1245
OFFSET_MIDDLE_SHELL = 0.0864
OFFSET_INNER_SHELL = 0.0525
LEFT_STORAGE_OFFSET = [-0.0705, 0.1017, 0.05, 1]
RIGHT_STORAGE_OFFSET = [0.485, 0.1017, 0.05, 1]
LEFT_REMOVED = [-0.0705, -0.01, 0.05, 1]
RIGHT_REMOVED = [0.485, -0.01, 0.05, 1]
STORAGE_OFFSET = 0.0375
ROBOT_TO_TRACKING = np.array(pd.read_csv('~/floating_ws/robot_to_tracking_system.csv'))
ORIENTATION = [0.0, 1.0, 0.0, 0.0]


class TfClient(Node):
    def __init__(self):
        # initialisation of the subscriber
        super().__init__('tf_client')
        self.cli = self.create_client(TrackingData, 'tf_data')
        self.req = TrackingData.Request()

    def send_request(self, get):
        self.req.name = get
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().position, self.future.result().rotation


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


class VacuumClient(Node):
    def __init__(self):
        super().__init__('vacuum_client')
        self.cli = self.create_client(SetUrIo, 'vacuum')
        self.req = SetUrIo.Request()

    def send_request(self, state):
        self.req.set = state
        self.cli.wait_for_service()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().current


class BoardTrackingClient(Node):
    def __init__(self):
        super().__init__('board_tracking_client')
        self.cli = self.create_client(TrackingData, 'board_pose')
        self.req = TrackingData.Request()

    def send_request(self, name):
        self.req.name = name
        self.cli.wait_for_service()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().position, self.future.result().rotation


def get_robot_position():
    rclpy.init()
    listener = TfClient()
    pos = listener.send_request('get')[0]
    return pos


def lower_to_piece(control, location):
    pos = get_robot_position()
    if location == 'board':
        pos[2] = 0.08
    else:
        pos[2] = 0.03
    control.send_request(pos, ORIENTATION, True)


def get_board_from_tracking():
    rclpy.init()
    tracking = BoardTrackingClient()
    rot_board = Rot.from_quat(tracking.send_request('board')[1]).as_matrix()
    pos_board = tracking.send_request('board')[0]
    tracking_to_board = np.zeros((4, 4))
    tracking_to_board[:3, :3] = rot_board
    tracking_to_board[:3, -1] = pos_board
    tracking_to_board[3] = [0, 0, 0, 1]
    return tracking_to_board


def get_robot_to_board():
    tracking_to_board = get_board_from_tracking()
    robot_to_board = ROBOT_TO_TRACKING @ tracking_to_board
    return robot_to_board


def move_to_pos_form_board(control, pos):
    x_offset = 0
    y_offset = 0

    if pos.x == 0:
        shell_origin = SAFE_POS
        current_shell_offset = OFFSET_OUTER_SHELL
    elif pos.x == 1:
        shell_origin = MIDDLE_SHELL
        current_shell_offset = OFFSET_MIDDLE_SHELL
    else:
        shell_origin = INNER_SHELL
        current_shell_offset = OFFSET_INNER_SHELL
    if pos.y == 1 or pos.y == 5:
        x_offset = shell_origin[0] + current_shell_offset
    if 2 <= pos.y <= 4:
        x_offset = shell_origin[0] + 2 * current_shell_offset
    if pos.y == 3 or pos.y == 7:
        y_offset = shell_origin[1] + current_shell_offset
    if 4 <= pos.y <= 6:
        y_offset = shell_origin[1] + 2 * current_shell_offset

    # assuming board[0][0] is in origin of board tracking system:
    pos_on_board = get_robot_to_board() @ np.array([x_offset, y_offset, 0.05, 1]).T
    control.send_request(pos_on_board, ORIENTATION, True)


def move_to_board_save_pos(control):
    pos_on_board = get_robot_to_board() @ SAFE_POS.T
    control.send_request(pos_on_board, ORIENTATION, True)


def move_piece(pos_start, pos_end):
    rclpy.init()
    vacuum = VacuumClient()
    control = ControlClient()
    # 1. move to start pos
    move_to_pos_form_board(control, pos_start)
    # 2. pick up piece
    vacuum.send_request(1)
    lower_to_piece(control, 'board')
    # 3. move to end pos
    move_to_pos_form_board(control, pos_end)
    # 4. place piece
    lower_to_piece(control, 'board')
    vacuum.send_request(0)
    # 5. move to safe pos
    move_to_board_save_pos(control)


def move_to_next_available_storage(game, control):
    if game.player == 1:
        x_offset = RIGHT_STORAGE_OFFSET[0]
        y_offset = RIGHT_STORAGE_OFFSET[1]
        current_offset = -STORAGE_OFFSET
    else:
        x_offset = LEFT_STORAGE_OFFSET[0]
        y_offset = LEFT_STORAGE_OFFSET[1]
        current_offset = STORAGE_OFFSET
    x_offset += game.pieces_on_board[game.player] % 3 * current_offset
    y_offset += game.pieces_on_board[game.player] // 3 * current_offset

    pos_on_board = get_robot_to_board() @ np.array([x_offset, y_offset, 0, 1]).T
    control.send_request(pos_on_board, ORIENTATION, True)


def remove_piece(game, move):
    rclpy.init()
    control = ControlClient()
    move_to_pos_form_board(control, move.remove_pos)
    if game.player == 1:
        x_offset = RIGHT_REMOVED[0]
        y_offset = RIGHT_REMOVED[1]
        current_offset = -STORAGE_OFFSET
    else:
        x_offset = LEFT_REMOVED[0]
        y_offset = LEFT_REMOVED[1]
        current_offset = STORAGE_OFFSET
    x_offset += game.pieces_on_board[game.player] % 3 * current_offset
    y_offset += game.pieces_on_board[game.player] // 3 * current_offset

    pos_on_board = get_robot_to_board() @ np.array([x_offset, y_offset, 0.05, 1]).T
    control.send_request(pos_on_board, ORIENTATION, True)


def place_piece(game, move):
    rclpy.init()
    vacuum = VacuumClient()
    control = ControlClient()
    # board_rot = Rot.from_matrix(board_in_rob_coord[:3, :3]).as_quat()
    # 1. move to storage pos
    move_to_next_available_storage(game, control)
    # 2. pick up piece
    lower_to_piece(control, 'storage')
    vacuum.send_request(1)
    # 3. move to move.place_pos
    move_to_pos_form_board(control, move.place_pos)
    # 4. release piece
    lower_to_piece(control, 'storage')
    vacuum.send_request(0)
    # 5. move to safe pos
    move_to_board_save_pos(control)
