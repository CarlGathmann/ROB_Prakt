from morris_engine.morris_engine.robot_commands import *
from utils import *
from colorama import Fore


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
                place_piece(self, move)
            # Move piece
            if move.move_pos_start is not None and move.move_pos_end is not None:
                board[move.move_pos_start.x][move.move_pos_start.y] = 0
                board[move.move_pos_end.x][move.move_pos_end.y] = self.player
                # robot control method
                move_piece(move.move_pos_start, move.move_pos_end)
            if move.remove_pos is not None:
                board[move.remove_pos.x][move.remove_pos.y] = 0  # Remove piece
                self.pieces_on_board[self.get_opponent()] -= 1
                # robot control method
                remove_piece(self, move)
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


def main():
    game = NineMensMorris()

    while not game.game_over():
        move = game.build_move_human()
        game.execute_move(move)


if __name__ == '__main__':
    main()
