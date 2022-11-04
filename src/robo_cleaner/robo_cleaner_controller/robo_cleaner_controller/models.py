from enum import IntEnum
from typing import Tuple
from robo_cleaner_interfaces.msg import BatteryStatus

class ROBOT_MOVE_TYPE(IntEnum):
    FORWARD = 0
    ROTATE_LEFT = 1
    ROTATE_RIGHT = 2

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
    CHARGER = 64
    DIRT_0 = 48
    DIRT_1 = 49
    DIRT_2 = 50
    DIRT_3 = 51
    UNKNOWN = 0

    def is_reentrant_dirt(val):
        return val in [
            TILE_TYPE.DIRT_1,
            TILE_TYPE.DIRT_2,
            TILE_TYPE.DIRT_3,
        ]

    def is_unexplored(val):
        return val in [
            TILE_TYPE.DIRT_1,
            TILE_TYPE.DIRT_2,
            TILE_TYPE.DIRT_3,
            TILE_TYPE.UNKNOWN,
        ]

    def is_passable_and_unexplored(val):
        return  TILE_TYPE.is_passable(val) and TILE_TYPE.is_unexplored(val)

    def is_passable(val):
        return val not in [
            TILE_TYPE.SMALL_OBSTACLE,
            TILE_TYPE.BIG_OBSTACLE,
            TILE_TYPE.OUT_OF_BOUND
        ]

    def is_passable_and_explored(val):
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
    def __init__(self, left : int, forward : int, right : int, back : int):
        self.left = TILE_TYPE(left)
        self.forward = TILE_TYPE(forward)
        self.right = TILE_TYPE(right)
        self.back = TILE_TYPE(back)

    def __str__(self):
        return f"Left: {self.left}, Forward: {self.forward}, Right: {self.right}, Back: {self.back}"

    def __repr__(self):
        return f"Left: {self.left}, Forward: {self.forward}, Right: {self.right}, Back: {self.back}"

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

    def __str__(self):
        return f"next_step: {self.next_step}, move_type: {self.move_type}"

    def __repr__(self):
        return f"next_step: {self.next_step}, move_type: {self.move_type}"

class Coordinates:
    def __init__(self, row : int, column : int):
        self.row = row
        self.column = column

    def __str__(self):
        return f"Row: {self.row}, Column: {self.column}"

    def __repr__(self):
        return f"Row: {self.row}, Column: {self.column}"

    def asMapNode(self):
        return MapNode(self.row, self.column)
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

class Battery:
    def __init__(self):
        self.max_moves_on_full_energy = 0
        self.moves_left = 0

    def update(self, status : BatteryStatus):
        self.max_moves_on_full_energy = status.max_moves_on_full_energy
        self.moves_left = status.moves_left