from enum import IntEnum
from typing import Tuple

EXPLORED_MARKER = 1000

class ROBOT_MOVE_TYPE(IntEnum):
    FORWARD = 0
    ROTATE_LEFT = 1
    ROTATE_RIGHT = 2
    UNKNOWN = 3

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

class ROBOT_DIRECTION(IntEnum):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

class TILE_TYPE(IntEnum):
    SMALL_OBSTACLE = 120
    BIG_OBSTACLE = 88
    OUT_OF_BOUND = 35
    CRYSTAL_CYAN = 99
    CRYSTAL_PURPLE = 112
    CRYSTAL_BLUE = 98
    CRYSTAL_GREEN = 103
    CRYSTAL_RED = 114
    UNKNOWN = 0

    def is_unexplored(val):
        return val < EXPLORED_MARKER

    def is_passable_and_unexplored(val):
        return  TILE_TYPE.is_passable(val) and TILE_TYPE.is_unexplored(val)

    def is_passable(val):
        if val > EXPLORED_MARKER:
            val -= EXPLORED_MARKER

        return val not in [
            TILE_TYPE.SMALL_OBSTACLE,
            TILE_TYPE.BIG_OBSTACLE,
            TILE_TYPE.OUT_OF_BOUND,
            TILE_TYPE.UNKNOWN
        ]

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

class SurroundingTiles:
    def __init__(self, left : int, forward : int, right : int):
        self.left = TILE_TYPE(left)
        self.forward = TILE_TYPE(forward)
        self.right = TILE_TYPE(right)

    def __str__(self):
        return f"Left: {self.left}, Forward: {self.forward}, Right: {self.right}"

    def __repr__(self):
        return f"Left: {self.left}, Forward: {self.forward}, Right: {self.right}"

class MapMoveResult(IntEnum):
    CONTINUE = 0
    BACKTRACK = 1
    FINISH = 3

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

class MapUpdateResult:
    def __init__(self, next_step : MapMoveResult, move_type: ROBOT_MOVE_TYPE):
        self.next_step = next_step
        self.move_type = move_type

class MapNode:
    def __init__(self, row: int, column: int):
        self._row = row
        self._column = column

    def __str__(self):
        return f"({self._row}, {self._column})"

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return hash((self._row, self._column))

    def __eq__(self, other):
        return self._row == other._row and self._column == other._column

    @property
    def row(self):
        return self._row

    @property
    def column(self):
        return self._column

    def __init__(
        self,
        top_left : MapNode,
        top_right : MapNode,
        bottom_right : MapNode,
        bottom_left : MapNode
    ):
        self.top_left = top_left
        self.top_right = top_right
        self.bottom_left = bottom_left
        self.bottom_right = bottom_right

    def contains(self, node : MapNode) -> bool:
        return node.row <= self.top_left.row and \
            node.row >= self.bottom_left.row and \
            node.column >= self.top_left.column and \
            node.column <= self.top_right.column