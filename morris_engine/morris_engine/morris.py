from utils import *
from colorama import Fore
import time
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


def get_robot_position():
    rclpy.init()
    listener = TfClient()
    pos = listener.send_request('get')[0]
    return pos


def move_to_pos_form_board(control, pos):
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


def lower_to_piece(control, location):
    pos = get_robot_position()
    if location == 'board':
        pos[2] = 0.08
    else:
        pos[2] = 0.03
    control.send_request(pos, ORIENTATION, True)


def move_piece_robot(pos_start, pos_end):
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

class NineMensMorris:
    def __init__(self):
        self.player_pieces = {1: 9, 2: 9}
        self.pieces_on_board = {1: 0, 2: 0}
        self.player = 1  # Player 1 starts
        self.phase = 1  # Phase 1: Place pieces, Phase 2: Move pieces
        print_board()
        print(Fore.RED + "PHASE 1: PLACE PIECES" + Fore.RESET)

    def build_move_human(self):
        """ Build a move for the human player """
        move = Move()
        print("Player: ", Fore.BLUE + str(self.player) if self.player == 1 else Fore.RED + str(self.player))
        print(Fore.RESET)
        if self.phase == 1:
            move.place_pos = input_position("input position to place piece")
            if mill_formed(move, self.player):
                print("Mill formed. Please remove a piece from the opponent.")
                move.remove_pos = input_position("input position of opponent's piece to remove")
        else:
            move.move_pos_start = input_position("input position of piece to move")
            move.move_pos_end = input_position("input position to move piece to")
            if mill_formed(move, self.player):
                print("Mill formed. Please remove a piece from the opponent.")
                move.remove_pos = input_position("input position of opponent's piece to remove")
        return move

    def build_move_ai(self):
        """ Build a move for the AI player """
        move = Move()
        print("Player: ", self.player)
        #  TODO: Implement AI
        return move

    def check_move(self, move: Move):
        """ Check if a move is valid """
        if self.check_remove(move) and self.check_place(move) and self.check_moved_stone(move):
            return True  # Move is valid
        else:
            return False  # Move is invalid

    def check_moved_stone(self, move):
        if self.phase == 2:
            if get_board_entry(move.move_pos_start) == self.player:
                if get_board_entry(move.move_pos_end) == 0:
                    if move.move_pos_end.x - move.move_pos_start.x == 1 or \
                            move.move_pos_end.x - move.move_pos_start.x == -1 or \
                            move.move_pos_end.y - move.move_pos_start.y == 1 or \
                            move.move_pos_end.y - move.move_pos_start.y == -1:
                        return True  # start_pos is own piece, end_pos is empty and end_pos is adjacent to start pos
                    print("You can only move to an adjacent field.")
                    return False  # end_pos is not adjacent to start_pos
                print("You can only move to an empty field.")
                return False  # end_pos is not empty
            print("You can only move your own pieces.")
            return False  # start_pos is not own piece
        else:
            return True  # Phase 1: No need to check if move is valid

    def check_place(self, move: Move):
        if self.phase == 1:
            if get_board_entry(move.place_pos) == 0:
                return True  # Field is empty
            else:
                print("This field is not empty.")
                return False  # Field is occupied
        else:
            return True  # Phase 2: No need to check placement

    def check_remove(self, move: Move):
        """ Check if a piece can be removed """
        if move.remove_pos is not None:
            if get_board_entry(move.remove_pos) == self.get_opponent():
                if not (mill_formed(move, self.get_opponent()) and
                        self.pieces_on_board[self.get_opponent()] > 3):
                    return True  # Piece can be removed
                else:
                    print("This piece is part of your opponents mill. You can't remove it.")
                    return False  # Cannot remove piece from mill if opponent has more than 3 pieces on board
            else:
                print("You can only remove your opponent's pieces.")
                return False  # Cannot remove own piece
        else:
            return True  # No piece to remove

    def execute_move(self, move: Move):
        """ Execute a move """
        if self.check_move(move):
            if move.place_pos is not None:
                board[move.place_pos.x][move.place_pos.y] = self.player  # Place piece
                self.player_pieces[self.player] -= 1
                self.pieces_on_board[self.player] += 1
                # Robot control method
                self.place_piece_robot(move)
            # Move piece
            if move.move_pos_start is not None and move.move_pos_end is not None:
                board[move.move_pos_start.x][move.move_pos_start.y] = 0
                board[move.move_pos_end.x][move.move_pos_end.y] = self.player
                # robot control method
                move_piece_robot(move.move_pos_start, move.move_pos_end)
            if move.remove_pos is not None:
                board[move.remove_pos.x][move.remove_pos.y] = 0  # Remove piece
                self.pieces_on_board[self.get_opponent()] -= 1
                # robot control method
                self.remove_piece_robot(move.remove_pos)
            print_board()
            if self.player_pieces[self.player] == 0 and \
                    self.player_pieces[self.get_opponent()] == 0 and self.phase == 1:
                self.phase = 2
                print(Fore.RED + "PHASE 2: MOVE PIECES" + Fore.RESET)
            self.player = self.get_opponent()
        else:
            # TODO: check if Human or AI
            self.execute_move(self.build_move_human())

    def get_opponent(self):
        return (self.player % 2) + 1  # Switch player

    def game_over(self):
        """ Check if the game is over """
        if self.phase == 2 and (self.pieces_on_board[1] < 3 or self.pieces_on_board[2] < 3):
            print(Fore.RED + "GAME OVER!")
            print("PLAYER " + str(max(self.pieces_on_board, key=self.pieces_on_board.get)) + " WINS!" + Fore.RESET)
            return True
        return False

    def place_piece_robot(self, move):
        rclpy.init()
        vacuum = VacuumClient()
        control = ControlClient()
        # board_rot = Rot.from_matrix(board_in_rob_coord[:3, :3]).as_quat()
        # 1. move to storage pos
        self.move_to_next_available_storage(control)
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

    def move_to_next_available_storage(self, control):
        if self.player == 1:
            x_offset = RIGHT_STORAGE_OFFSET[0]
            y_offset = RIGHT_STORAGE_OFFSET[1]
            current_offset = -STORAGE_OFFSET
        else:
            x_offset = LEFT_STORAGE_OFFSET[0]
            y_offset = LEFT_STORAGE_OFFSET[1]
            current_offset = STORAGE_OFFSET
        x_offset += self.pieces_on_board[self.player] % 3 * current_offset
        y_offset += self.pieces_on_board[self.player] // 3 * current_offset

        pos_on_board = get_robot_to_board() @ np.array([x_offset, y_offset, 0, 1]).T
        control.send_request(pos_on_board, ORIENTATION, True)

    def remove_piece_robot(self, remove_pos):
        rclpy.init()
        vacuum = VacuumClient()
        control = ControlClient()
        move_to_pos_form_board(control, remove_pos)
        if self.player == 1:
            x_offset = RIGHT_REMOVED[0]
            y_offset = RIGHT_REMOVED[1]
            current_offset = -STORAGE_OFFSET
        else:
            x_offset = LEFT_REMOVED[0]
            y_offset = LEFT_REMOVED[1]
            current_offset = STORAGE_OFFSET
        x_offset += self.pieces_on_board[self.player] % 3 * current_offset
        y_offset += self.pieces_on_board[self.player] // 3 * current_offset

        pos_on_board = get_robot_to_board() @ np.array([x_offset, y_offset, 0.05, 1]).T
        control.send_request(pos_on_board, ORIENTATION, True)


def main():
    game = NineMensMorris()

    while not game.game_over():
        move = game.build_move_human()
        game.execute_move(move)


if __name__ == '__main__':
    main()
