from copy import deepcopy
from colorama import Fore


class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Move:
    def __init__(self):
        self.place_pos = None
        self.remove_pos = None
        self.move_pos_start = None
        self.move_pos_end = None


board = [[0 for _ in range(8)] for _ in range(3)]  # Initialize empty board
#board = [[1, 1, 0, 1, 0, 0, 0, 0], [2, 2, 2, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0]]


def get_board_entry(pos):
    return board[pos.x][pos.y]


def mill_formed(move: Move, player) -> bool:
    """ Check if a mill (3 pieces in a row) has been formed """
    # Check corners

    temp = deepcopy(board)

    if move.place_pos is not None:
        pos = move.place_pos
        temp[move.place_pos.x][move.place_pos.y] = player
    elif move.move_pos_start is not None and move.move_pos_end is not None:
        pos = move.move_pos_end
        temp[move.move_pos_start.x][move.move_pos_start.y] = 0
        temp[move.move_pos_end.x][move.move_pos_end.y] = player
    else:
        return False  # invalid: position not set

    if pos.y % 2 == 0:
        if temp[pos.x][pos.y] and temp[pos.x][(pos.y + 1) % 8] == temp[pos.x][(pos.y + 2) % 8] == player or \
                temp[pos.x][pos.y] and temp[pos.x][pos.y - 1] == temp[pos.x][pos.y - 2] == player:
            return True  # Corner mill
    else:
        if temp[(pos.x + 1) % 3][pos.y] == temp[pos.x - 1][pos.y] == player:
            return True  # Vertical mill
        if temp[pos.x][(pos.y + 1) % 8] == temp[pos.x][pos.y - 1] == player:
            return True  # Horizontal mill
    return False


def input_position(string) -> Position:
    """ Input a position from the user """
    print(string,)
    while True:
        x = input("Enter x: ")
        y = input("Enter y: ")
        if x.isdigit() and y.isdigit() and int(x) < 3 and int(y) < 8:
            return Position(int(x), int(y))
        else:
            print("Invalid input. Please try again.")


def print_board():
    temp = deepcopy(board)
    for i in range(3):
        for j in range(8):
            temp[i][j] = str(temp[i][j])
            if temp[i][j] == "1":
                temp[i][j] = Fore.BLUE + "1" + Fore.RESET
            elif temp[i][j] == "2":
                temp[i][j] = Fore.RED + "2" + Fore.RESET
    print(temp[0][0], "(0,0)-------------------------", temp[0][1],
          "(0,1)--------------------------", temp[0][2], "(0,2)")
    print("|                                |                                 |")
    print("|                                |                                 |")
    print("|                                |                                 |")
    print("|         ", temp[1][0], "(1,0)--------------",
          temp[1][1], "(1,1)---------------", temp[1][2], "(1,2)    |")
    print("|          |                     |                      |          |")
    print("|          |                     |                      |          |")
    print("|          |                     |                      |          |")
    print("|          |         ", temp[2][0], "(2,0)---",
          temp[2][1], "(2,1)----", temp[2][2], "(2,2)    |          |")
    print("|          |          |                      |          |          |")
    print("|          |          |                      |          |          |")
    print("|          |          |                      |          |          |")
    print(temp[0][7], "(0,7)---", temp[1][7], "(1,7)---", temp[2][7], "(2,7)               ",
          temp[2][3], "(2,3)---", temp[1][3], "(1,3)---", temp[0][3], "(0,3)")
    print("|          |          |                      |          |          |")
    print("|          |          |                      |          |          |")
    print("|          |          |                      |          |          |")
    print("|          |         ", temp[2][6],  "(2,6)---",
          temp[2][5], "(2,5)----", temp[2][4], "(2,4)    |          |")
    print("|          |                     |                      |          |")
    print("|          |                     |                      |          |")
    print("|          |                     |                      |          |")
    print("|         ", temp[1][6], "(1,6)--------------",
          temp[1][5], "(1,5)---------------", temp[1][4], "(1,4)    |")
    print("|                                |                                 |")
    print("|                                |                                 |")
    print("|                                |                                 |")
    print(temp[0][6], "(0,6)-------------------------", temp[0][5],
          "(0,5)--------------------------", temp[0][4], "(0,4)")
    print("\n")
