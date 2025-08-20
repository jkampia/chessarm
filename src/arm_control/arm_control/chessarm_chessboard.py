from enum import Enum, IntEnum
import numpy as np


class Occupancy(IntEnum):
    EMPTY = 0
    WHITE = 1
    BLACK = 2


class PieceType(IntEnum):
    PAWN = 0
    ROOK = 1
    KNIGHT = 2
    BISHOP = 3
    QUEEN = 4
    KING = 5


StartingPieceOrder = {
    1: [PieceType.ROOK, PieceType.KNIGHT, PieceType.BISHOP, PieceType.QUEEN, PieceType.KING, PieceType.BISHOP, PieceType.KNIGHT, PieceType.ROOK],
    2: [PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN],
    3: [None, None, None, None, None, None, None, None],
    4: [None, None, None, None, None, None, None, None],
    5: [None, None, None, None, None, None, None, None],
    6: [None, None, None, None, None, None, None, None],
    7: [PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN, PieceType.PAWN],
    8: [PieceType.ROOK, PieceType.KNIGHT, PieceType.BISHOP, PieceType.QUEEN, PieceType.KING, PieceType.BISHOP, PieceType.KNIGHT, PieceType.ROOK]
}


class ChessSquare:

    def __init__(self):

        self.occupancy = Occupancy.EMPTY
        self.piece_type = None
        


class ChessBoard:

    def __init__(self):
        
        self.current_state = [[ChessSquare() for col in range(8)] for row in range(8)]
        self.setupInitialBoard("white")
        

    def setupInitialBoard(self, robot_color):
        if robot_color == "white":
            for i in range(8):
                for j in range(8):
                    self.current_state[i][j].piece_type = StartingPieceOrder[i+1][j]
                    print(self.current_state[i][j].piece_type)
                



board = ChessBoard()

