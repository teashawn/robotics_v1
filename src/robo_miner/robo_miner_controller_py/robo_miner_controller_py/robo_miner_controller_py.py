from enum import IntEnum
import numpy as np

from robo_miner_interfaces.srv import QueryInitialRobotPosition, RobotMove
from robo_miner_interfaces.msg import RobotMoveType
import rclpy
from rclpy.node import Node

from pprint import pprint

EXPLORED_MARKER = 1000
TRIGGER = True

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
    #PLAYER = 66
    #ENEMY = 69
    SMALL_OBSTACLE = 120
    BIG_OBSTACLE = 88
    #EMPTY = 46
    OUT_OF_BOUND = 35
    CRYSTAL_CYAN = 99
    CRYSTAL_PURPLE = 112
    CRYSTAL_BLUE = 98
    CRYSTAL_GREEN = 103
    CRYSTAL_RED = 114

    def is_unexplored(val):
        return val < EXPLORED_MARKER

    def is_passable_and_unexplored(val):
        if TRIGGER:
            print(f"val: {val}, passable: {TILE_TYPE.is_passable(val)}, unexplored: {TILE_TYPE.is_unexplored(val)}, both: {TILE_TYPE.is_passable(val) and TILE_TYPE.is_unexplored(val)}")
        return  TILE_TYPE.is_passable(val) and TILE_TYPE.is_unexplored(val)

    def is_passable(val):
        if val > EXPLORED_MARKER:
            val -= EXPLORED_MARKER

        return val not in [
            TILE_TYPE.SMALL_OBSTACLE,
            TILE_TYPE.BIG_OBSTACLE,
            TILE_TYPE.OUT_OF_BOUND
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

class QueryInitialRobotPositionClientAsync(Node):

    def __init__(self):
        super().__init__('query_initial_robot_position_client_async')
        self.cli = self.create_client(QueryInitialRobotPosition, 'query_initial_robot_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = QueryInitialRobotPosition.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def query(self):
        response = self.send_request()

        # Log
        success = response.robot_position_response.success
        error_reason = response.robot_position_response.error_reason
        surrounding_tiles = SurroundingTiles(*response.robot_position_response.surrounding_tiles)
        robot_dir = ROBOT_DIRECTION(response.robot_position_response.robot_dir)
        self.get_logger().info(
            f"\nSuccess: {success},\nError reason: {error_reason},\nSurrounding tiles: {surrounding_tiles},\nDirection: {robot_dir}\n"
        )
        
        return response #.robot_position_response

class RobotMoveClientAsync(Node):

    def __init__(self):
        super().__init__('robot_move_client_async')
        self.cli = self.create_client(RobotMove, 'move_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, move_type : ROBOT_MOVE_TYPE):
        req = RobotMove.Request(robot_move_type=RobotMoveType(move_type=move_type))
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def move(self, move_type : ROBOT_MOVE_TYPE):
        move_response = self.send_request(move_type)

        # Log
        success = move_response.robot_position_response.success
        error_reason = move_response.robot_position_response.error_reason
        surrounding_tiles = SurroundingTiles(*move_response.robot_position_response.surrounding_tiles)
        robot_dir = ROBOT_DIRECTION(move_response.robot_position_response.robot_dir)
        self.get_logger().info(
            f"\nSuccess: {success},\nError reason: {error_reason},\nSurrounding tiles: {surrounding_tiles},\nDirection: {robot_dir}\n"
        )

        return move_response.robot_position_response

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

class MapExplorer:
    def __init__(self, debug : bool):
        self.INITIAL_TILE : TILE_TYPE = None
        self.DIRECTION : ROBOT_DIRECTION = None
        self.SURROUNDING_TILES : SurroundingTiles = None
        self.ROW = 0
        self.COLUMN = 0
        self.MAP = np.zeros((1,1), dtype=int)
        self.UNEXPLORED_TILES = 0
        self.DEBUG = debug
        # np.pad(a, ((prepend_rows, append_rows),(prepend_columns, append_columns)))
        # e.g. np.pad(a, ((0,1), (0,0))) to append 1 row

    def print_state(self):
        print("####################################")
        print(f"# Map Explorer State:")
        print(f"# Row: {self.ROW}, Column: {self.COLUMN}, Unexplored: {self.UNEXPLORED_TILES}, Map size: {self.MAP.shape}")
        pprint(self.MAP)
        print("####################################")

    def init(self, response) -> MapUpdateResult:
        self.INITIAL_TILE = TILE_TYPE(response.robot_initial_tile)
        self.MAP[0,0] = self.INITIAL_TILE.value + EXPLORED_MARKER

        return self.update(ROBOT_MOVE_TYPE.UNKNOWN, response.robot_position_response)

    def _update_position(self) -> None:
        if self.DIRECTION == ROBOT_DIRECTION.DOWN:
            self.ROW += 1
        elif self.DIRECTION == ROBOT_DIRECTION.UP:
            self.ROW -= 1
        elif self.DIRECTION == ROBOT_DIRECTION.LEFT:
            self.COLUMN -= 1
        elif self.DIRECTION == ROBOT_DIRECTION.RIGHT:
            self.COLUMN += 1

    def _handle_direction_up(self):
        map_rows, map_columns = self.MAP.shape

        if self.SURROUNDING_TILES.forward != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.ROW == 0:
                # we're at the top corner of the explored map
                # so we prepend a row
                self.MAP = np.pad(self.MAP, ((1,0),(0,0)))
                # update current row index
                self.ROW += 1

                if self.DEBUG:
                    print(f"Prepending a row")
            # store tile value
            if self.MAP[self.ROW-1, self.COLUMN] == 0:
                self.MAP[self.ROW-1, self.COLUMN] = self.SURROUNDING_TILES.forward.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.forward):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.forward = self.MAP[self.ROW-1, self.COLUMN]
        if self.SURROUNDING_TILES.left != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.COLUMN == 0:
                # we're at the left corner of the explored map
                # so we prepend a column
                self.MAP = np.pad(self.MAP, ((0,0),(1,0)))
                # update current column index
                self.COLUMN += 1

                if self.DEBUG:
                    print(f"Prepending a column")
            # store tile value
            if self.MAP[self.ROW, self.COLUMN-1] == 0:
                self.MAP[self.ROW, self.COLUMN-1] = self.SURROUNDING_TILES.left.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.left):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.left = self.MAP[self.ROW, self.COLUMN-1]
        if self.SURROUNDING_TILES.right != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.COLUMN+1 == map_columns:
                # we're at the right corner of the explored map
                # so we append a column
                self.MAP = np.pad(self.MAP, ((0,0),(0,1)))
            # store tile value
            if self.MAP[self.ROW, self.COLUMN+1] == 0:
                self.MAP[self.ROW, self.COLUMN+1] = self.SURROUNDING_TILES.right.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.right):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.right = self.MAP[self.ROW, self.COLUMN+1]

    def _handle_direction_down(self):
        map_rows, map_columns = self.MAP.shape

        if self.SURROUNDING_TILES.forward != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.ROW+1 == map_rows:
                # we're at the bottom corner of the explored map
                # so we append a row
                self.MAP = np.pad(self.MAP, ((0,1),(0,0)))
            # store tile value
            if self.MAP[self.ROW+1, self.COLUMN] == 0:
                self.MAP[self.ROW+1, self.COLUMN] = self.SURROUNDING_TILES.forward.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.forward):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.forward = self.MAP[self.ROW+1, self.COLUMN]
        if self.SURROUNDING_TILES.left != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.COLUMN+1 == map_columns:
                # we're at the right corner of the explored map
                # so we append a column
                self.MAP = np.pad(self.MAP, ((0,0),(0,1)))
            # store tile value
            if self.MAP[self.ROW, self.COLUMN+1] == 0:
                self.MAP[self.ROW, self.COLUMN+1] = self.SURROUNDING_TILES.left.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.left):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.left = self.MAP[self.ROW, self.COLUMN+1]
        if self.SURROUNDING_TILES.right != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.COLUMN == 0:
                # we're at the left corner of the explored map
                # so we prepend a column
                self.MAP = np.pad(self.MAP, ((0,0),(1,0)))
                # update current column index
                self.COLUMN += 1

                if self.DEBUG:
                    print(f"Prepending a column")
            # store tile value
            if self.MAP[self.ROW, self.COLUMN-1] == 0:
                self.MAP[self.ROW, self.COLUMN-1] = self.SURROUNDING_TILES.right.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.right):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.right = self.MAP[self.ROW, self.COLUMN-1]

    def _handle_direction_left(self):
        map_rows, map_columns = self.MAP.shape

        if self.SURROUNDING_TILES.forward != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.COLUMN == 0:
                # we're at the left corner of the explored map
                # so we prepend a column
                self.MAP = np.pad(self.MAP, ((0,0),(1,0)))
                # update current column index
                self.COLUMN += 1

                if self.DEBUG:
                    print(f"Prepending a column")
            # store tile value
            if self.MAP[self.ROW, self.COLUMN-1] == 0:
                self.MAP[self.ROW, self.COLUMN-1] = self.SURROUNDING_TILES.forward.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.forward):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.forward = self.MAP[self.ROW, self.COLUMN-1]
        if self.SURROUNDING_TILES.left != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.ROW+1 == map_rows:
                # we're at the bottom corner of the explored map
                # so we append a row
                self.MAP = np.pad(self.MAP, ((0,1),(0,0)))
            # store tile value
            if self.MAP[self.ROW+1, self.COLUMN] == 0:
                self.MAP[self.ROW+1, self.COLUMN] = self.SURROUNDING_TILES.left.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.left):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.left = self.MAP[self.ROW+1, self.COLUMN]
        if self.SURROUNDING_TILES.right != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.ROW == 0:
                # we're at the top corner of the explored map
                # so we prepend a row
                self.MAP = np.pad(self.MAP, ((1,0),(0,0)))
                # update current row index
                self.ROW += 1

                if self.DEBUG:
                    print(f"Prepending a row")
            # store tile value
            if self.MAP[self.ROW-1, self.COLUMN] == 0:
                self.MAP[self.ROW-1, self.COLUMN] = self.SURROUNDING_TILES.right.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.right):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.right = self.MAP[self.ROW-1, self.COLUMN]

    def _handle_direction_right(self):
        map_rows, map_columns = self.MAP.shape

        if self.SURROUNDING_TILES.forward != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.COLUMN+1 == map_columns:
                # we're at the right corner of the explored map
                # so we append a column
                self.MAP = np.pad(self.MAP, ((0,0),(0,1)))
            # store tile value
            if self.MAP[self.ROW, self.COLUMN+1] == 0:
                self.MAP[self.ROW, self.COLUMN+1] = self.SURROUNDING_TILES.forward.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.forward):
                    self.UNEXPLORED_TILES += 1
        if self.SURROUNDING_TILES.left != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.ROW == 0:
                # we're at the top corner of the explored map
                # so we prepend a row
                self.MAP = np.pad(self.MAP, ((1,0),(0,0)))
                # update current row index
                self.ROW += 1

                if self.DEBUG:
                    print(f"Prepending a row")
            # store tile value
            if self.MAP[self.ROW-1, self.COLUMN] == 0:
                self.MAP[self.ROW-1, self.COLUMN] = self.SURROUNDING_TILES.left.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.left):
                    self.UNEXPLORED_TILES += 1
        if self.SURROUNDING_TILES.right != TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.ROW+1 == map_rows:
                # we're at the bottom corner of the explored map
                # so we append a row
                self.MAP = np.pad(self.MAP, ((0,1),(0,0)))
            # store tile value
            if self.MAP[self.ROW+1, self.COLUMN] == 0:
                self.MAP[self.ROW+1, self.COLUMN] = self.SURROUNDING_TILES.right.value
                if TILE_TYPE.is_passable(self.SURROUNDING_TILES.right):
                    self.UNEXPLORED_TILES += 1

    def _update_map(self):
        map_rows, map_columns = self.MAP.shape

        if self.DIRECTION == ROBOT_DIRECTION.UP:
            self._handle_direction_up()
        elif self.DIRECTION == ROBOT_DIRECTION.DOWN:
            self._handle_direction_down()
        elif self.DIRECTION == ROBOT_DIRECTION.LEFT:
            self._handle_direction_left()
        elif self.DIRECTION == ROBOT_DIRECTION.RIGHT:
            self._handle_direction_right()

    def _detect_unexplored(self):
        return TILE_TYPE.is_passable_and_unexplored(self.SURROUNDING_TILES.forward) \
            or TILE_TYPE.is_passable_and_unexplored(self.SURROUNDING_TILES.left) \
            or TILE_TYPE.is_passable_and_unexplored(self.SURROUNDING_TILES.right)

    def update(self, move_type : ROBOT_MOVE_TYPE, response) -> MapUpdateResult:
        if response.success and move_type == ROBOT_MOVE_TYPE.FORWARD:
            # we moved to a new tile, as opposed to only changing direction
            self._update_position()
            if TILE_TYPE.is_passable_and_unexplored(self.MAP[self.ROW, self.COLUMN]):
                self.MAP[self.ROW, self.COLUMN] += EXPLORED_MARKER
                self.UNEXPLORED_TILES -= 1
    
        self.DIRECTION = ROBOT_DIRECTION(response.robot_dir)
        # TODO: get surrounding value from explored map, is available
        self.SURROUNDING_TILES = SurroundingTiles(*response.surrounding_tiles)

        self._update_map()

        if self.DEBUG:
            self.print_state()
            print("\n" + chr(176)*70)
            # TODO: remove
            #input()

        check_tile = TILE_TYPE.is_passable_and_unexplored if self._detect_unexplored() else TILE_TYPE.is_passable

        if check_tile(self.SURROUNDING_TILES.forward):
            return MapUpdateResult(
                next_step=MapMoveResult.CONTINUE,
                move_type=ROBOT_MOVE_TYPE.FORWARD)
        elif check_tile(self.SURROUNDING_TILES.right):
            return MapUpdateResult(
                next_step=MapMoveResult.CONTINUE,
                move_type=ROBOT_MOVE_TYPE.ROTATE_RIGHT)
        elif check_tile(self.SURROUNDING_TILES.left):
            return MapUpdateResult(
                next_step=MapMoveResult.CONTINUE,
                move_type=ROBOT_MOVE_TYPE.ROTATE_LEFT)
        else:
            if self.UNEXPLORED_TILES > 0:
                return MapUpdateResult(
                    next_step=MapMoveResult.BACKTRACK,
                    move_type=ROBOT_MOVE_TYPE.FORWARD)
            else:
                return MapUpdateResult(
                    next_step=MapMoveResult.FINISH,
                    move_type=ROBOT_MOVE_TYPE.UNKNOWN
                )

def main(args=None):
    rclpy.init(args=args)

    # State
    explorer = MapExplorer(True)
    explorer.print_state()

    # Service clients
    query_intial_robot_position_client = QueryInitialRobotPositionClientAsync()
    robot_move_client = RobotMoveClientAsync()
    
    # Get initial position
    response = query_intial_robot_position_client.query()
    res = explorer.init(response)

    while res.next_step != MapMoveResult.FINISH:
        if res.next_step == MapMoveResult.BACKTRACK:
            # Turn around
            move_response = robot_move_client.move(ROBOT_MOVE_TYPE.ROTATE_RIGHT)
            res = explorer.update(ROBOT_MOVE_TYPE.ROTATE_RIGHT, move_response)
            move_response = robot_move_client.move(ROBOT_MOVE_TYPE.ROTATE_RIGHT)
            res = explorer.update(ROBOT_MOVE_TYPE.ROTATE_RIGHT, move_response)
        else:
            move_response = robot_move_client.move(res.move_type)
            res = explorer.update(res.move_type, move_response)

    # Shut down
    query_intial_robot_position_client.destroy_node()
    robot_move_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()