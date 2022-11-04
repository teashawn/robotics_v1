from robo_cleaner_controller import models, service_clients, algo
from robo_cleaner_interfaces.msg import RobotMoveType
import numpy as np
import networkx as nx
from typing import (
    List,
    Dict,
    Tuple,
    Optional
)
from pprint import pprint
from autologging import traced
import time

"""
direction: move -> new direction

1-st level keys designate the current robot direction
2-nd level keys are the move of the robot

The innermost values are the resulting directions
"""
ROTATION_MAP = {
    models.ROBOT_DIRECTION.UP: {
        models.ROBOT_MOVE_TYPE.FORWARD: models.ROBOT_DIRECTION.UP,
        models.ROBOT_MOVE_TYPE.ROTATE_LEFT: models.ROBOT_DIRECTION.LEFT,
        models.ROBOT_MOVE_TYPE.ROTATE_RIGHT: models.ROBOT_DIRECTION.RIGHT,
    },
    models.ROBOT_DIRECTION.RIGHT: {
        models.ROBOT_MOVE_TYPE.FORWARD: models.ROBOT_DIRECTION.RIGHT,
        models.ROBOT_MOVE_TYPE.ROTATE_LEFT: models.ROBOT_DIRECTION.UP,
        models.ROBOT_MOVE_TYPE.ROTATE_RIGHT: models.ROBOT_DIRECTION.DOWN,
    },
    models.ROBOT_DIRECTION.DOWN: {
        models.ROBOT_MOVE_TYPE.FORWARD: models.ROBOT_DIRECTION.DOWN,
        models.ROBOT_MOVE_TYPE.ROTATE_LEFT: models.ROBOT_DIRECTION.RIGHT,
        models.ROBOT_MOVE_TYPE.ROTATE_RIGHT: models.ROBOT_DIRECTION.LEFT,
    },
    models.ROBOT_DIRECTION.LEFT: {
        models.ROBOT_MOVE_TYPE.FORWARD: models.ROBOT_DIRECTION.LEFT,
        models.ROBOT_MOVE_TYPE.ROTATE_LEFT: models.ROBOT_DIRECTION.DOWN,
        models.ROBOT_MOVE_TYPE.ROTATE_RIGHT: models.ROBOT_DIRECTION.UP,
    },
}

"""
orientation: target -> rotations

1-st level keys designate the current robot direction
2-nd level keys are the desired direction of the robot

The innermost values are the list of turns that need
to be performed in order to reach the target direction
"""
REORIENTATION_MAP = {
    models.ROBOT_DIRECTION.UP: {
        models.ROBOT_DIRECTION.UP: [

        ],
        models.ROBOT_DIRECTION.RIGHT: [
            models.ROBOT_MOVE_TYPE.ROTATE_RIGHT
        ],
        models.ROBOT_DIRECTION.DOWN: [
            models.ROBOT_MOVE_TYPE.ROTATE_RIGHT,
            models.ROBOT_MOVE_TYPE.ROTATE_RIGHT,
        ],
        models.ROBOT_DIRECTION.LEFT: [
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT
        ],
    },
    models.ROBOT_DIRECTION.RIGHT: {
        models.ROBOT_DIRECTION.UP: [
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT
        ],
        models.ROBOT_DIRECTION.RIGHT: [
            
        ],
        models.ROBOT_DIRECTION.DOWN: [
            models.ROBOT_MOVE_TYPE.ROTATE_RIGHT,
        ],
        models.ROBOT_DIRECTION.LEFT: [
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT,
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT
        ],
    },
    models.ROBOT_DIRECTION.DOWN: {
        models.ROBOT_DIRECTION.UP: [
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT,
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT
        ],
        models.ROBOT_DIRECTION.RIGHT: [
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT
        ],
        models.ROBOT_DIRECTION.DOWN: [
            
        ],
        models.ROBOT_DIRECTION.LEFT: [
            models.ROBOT_MOVE_TYPE.ROTATE_RIGHT,
        ],
    },
    models.ROBOT_DIRECTION.LEFT: {
        models.ROBOT_DIRECTION.UP: [
            models.ROBOT_MOVE_TYPE.ROTATE_RIGHT
        ],
        models.ROBOT_DIRECTION.RIGHT: [
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT,
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT
        ],
        models.ROBOT_DIRECTION.DOWN: [
            models.ROBOT_MOVE_TYPE.ROTATE_LEFT
        ],
        models.ROBOT_DIRECTION.LEFT: [
            
        ],
    },
}

@traced(
    'reveal_map',
    'navigate',
    'move',
    '_move_completed',
    'return_to_charger',
    'charge'
)
class MapExplorer:
    def __init__(self, debug : bool, use_turn_aware_pathfinding: bool, retardation : float = 0):
        self.DIRECTION : models.ROBOT_DIRECTION = None
        self.ROW = 1
        self.COLUMN = 1
        self.UNEXPLORED_TILES = 8
        self.MOVES = 0
        self.SURROUNDING_TILES = None
        self.LAST_MOVE_TYPE = None
        self.LAST_MOVE_CANCELLED = False
        self.CHARGER_LOCATION = models.Coordinates(1,1)
        self.BATTERY = models.Battery()
        self.DEBUG = debug
        self.NAVIGATING = False
        self.NAVIGATION_TARGET = None
        self.TURN_AWARE = use_turn_aware_pathfinding
        self.RETARDATION = retardation

        self.MAP = np.zeros((3,3), dtype=int)
        self.GRAPH = nx.Graph()

        self.initial_state_client = service_clients.QueryInitialRobotStateClientAsync(debug)
        self.authenticator = service_clients.Authenticator()
        self.battery_status_client = service_clients.BatteryStatusClientAsync(debug)
        self.charge_battery_client = service_clients.ChargeBatteryClientAsync(debug)
        self.move_counter_listener = service_clients.RobotMoveCounterSubscriber(debug, self._move_counter_updated)
        self.robot_move_client = service_clients.RobotMoveActionClient(
            debug=debug,
            cb_ack = self._move_acknowledged,
            cb_completed = self._move_completed,
            cb_feedback = self._move_feedback
        )

    def __del__(self):
        self.initial_state_client.destroy_node()
        self.authenticator.destroy_node()
        self.battery_status_client.destroy_node()
        self.charge_battery_client.destroy_node()
        self.move_counter_listener.destroy_node()

    def init(self) -> models.MapUpdateResult:
        response = self.initial_state_client.query()

        self.DIRECTION = models.ROBOT_DIRECTION(response.initial_robot_state.robot_dir)

        initial_tile= models.TILE_TYPE(response.initial_robot_state.robot_tile)
        self.MAP[1,1] = initial_tile.value
        if models.TILE_TYPE(initial_tile.value) == models.TILE_TYPE.CHARGER:
            self.CHARGER_LOCATION = models.Coordinates(1,1)

        battery_status = response.initial_robot_state.battery_status
        self.BATTERY.update(battery_status)
        
        self.SURROUNDING_TILES = self._get_surrounding_tiles()
        
        return self._explore_neighbours()

    ##################################################################
    # Callbacks
    ##################################################################

    def _move_counter_updated(self, msg):
        """
        For some reason the subscriber is not receiving data. To be debugged later.
        For now working around this by keeping our own counter.
        """
        #self.MOVES = msg.data
        pass

    def _move_acknowledged(self, goal_handle):
        if not goal_handle.accepted:
            if self.DEBUG:
                print("Goal rejected")
            return
        # TODO: remove if moves counter subscriber starts receiving data
        self.MOVES += 1

    def _move_feedback(self, msg):
        approaching_field_marker = msg.feedback.approaching_field_marker

        if approaching_field_marker > 0 and not models.TILE_TYPE.is_passable(approaching_field_marker):
            tile_ahead = self._get_coordinates_of_tile_ahead()
            
            # Special case for out-of-bounds marker. Because we know the map
            # is a rectangle, we mark the entire rectangle side as out of bound ;) 🕺🕺🕺
            if approaching_field_marker == models.TILE_TYPE.OUT_OF_BOUND:
                self._mark_map_border(tile_ahead)
            else:
                self.MAP[tile_ahead.row, tile_ahead.column] = approaching_field_marker

            if self.DEBUG:
                print(f"Cancelling move {tile_ahead}, because tile ahead is {models.TILE_TYPE(approaching_field_marker)}")

            self.robot_move_client.cancel_move()

    def _move_completed(self, result):
        if result.processed_field_marker == 0:
            # goal was cancelled
            self.SURROUNDING_TILES = self._get_surrounding_tiles()
            self._update_battery_status()
            self.LAST_MOVE_CANCELLED = True
            return

        if self.LAST_MOVE_TYPE == RobotMoveType.FORWARD:
            self._update_position()
            self.MAP[self.ROW, self.COLUMN] = result.processed_field_marker

            if self.DEBUG:
                print(f"Entered tile: {models.TILE_TYPE(result.processed_field_marker)}")
        
        self._update_map()
        self.DIRECTION = ROTATION_MAP[self.DIRECTION][self.LAST_MOVE_TYPE]
        self.SURROUNDING_TILES = self._get_surrounding_tiles()
        self._update_battery_status()
        
    def print_state(self):
        # Indicate current position on map via negative sign
        self.MAP[self.ROW, self.COLUMN] *= -1
        print("#" * 70)
        print(f"# Map Explorer State:")
        print(f"# Row: {self.ROW}, Column: {self.COLUMN}, Direction: {self.DIRECTION}, Unexplored: {self.UNEXPLORED_TILES}")
        print(f"# Charger: {self.CHARGER_LOCATION}")
        print(f"# Map size: {self.MAP.shape}, Moves: {self.MOVES}")
        print(f"# Navigating: {self.NAVIGATING}, Target: {self.NAVIGATION_TARGET}")
        print(f"# Battery: {self.BATTERY.moves_left}/{self.BATTERY.max_moves_on_full_energy}, Moves to charger: {len(self._get_moves_to_charger())}")
        pprint(self.MAP)
        print("#" * 70)
        # Restore current position tile value
        self.MAP[self.ROW, self.COLUMN] *= -1

    ##################################################################
    # Neighbour awareness
    ##################################################################

    def _get_coordinates_of_tile_ahead(self) -> models.MapNode:
        if self.DIRECTION == models.ROBOT_DIRECTION.UP:
            return models.MapNode(self.ROW-1, self.COLUMN)
        elif self.DIRECTION == models.ROBOT_DIRECTION.RIGHT:
            return models.MapNode(self.ROW, self.COLUMN+1)
        elif self.DIRECTION == models.ROBOT_DIRECTION.DOWN:
            return models.MapNode(self.ROW+1, self.COLUMN)
        elif self.DIRECTION == models.ROBOT_DIRECTION.LEFT:
            return models.MapNode(self.ROW, self.COLUMN-1)
        else:
            raise Exception(f"Invalid robot direction: {self.DIRECTION}")

    def _get_coordinates_of_tile_behind(self) -> models.MapNode:
        if self.DIRECTION == models.ROBOT_DIRECTION.UP:
            return models.MapNode(self.ROW+1, self.COLUMN)
        elif self.DIRECTION == models.ROBOT_DIRECTION.RIGHT:
            return models.MapNode(self.ROW, self.COLUMN-1)
        elif self.DIRECTION == models.ROBOT_DIRECTION.DOWN:
            return models.MapNode(self.ROW-1, self.COLUMN)
        elif self.DIRECTION == models.ROBOT_DIRECTION.LEFT:
            return models.MapNode(self.ROW, self.COLUMN+1)
        else:
            raise Exception(f"Invalid robot direction: {self.DIRECTION}")

    def _get_coordinates_of_tile_on_the_right(self) -> models.MapNode:
        if self.DIRECTION == models.ROBOT_DIRECTION.UP:
            return models.MapNode(self.ROW, self.COLUMN+1)
        elif self.DIRECTION == models.ROBOT_DIRECTION.RIGHT:
            return models.MapNode(self.ROW+1, self.COLUMN)
        elif self.DIRECTION == models.ROBOT_DIRECTION.DOWN:
            return models.MapNode(self.ROW, self.COLUMN-1)
        elif self.DIRECTION == models.ROBOT_DIRECTION.LEFT:
            return models.MapNode(self.ROW-1, self.COLUMN)
        else:
            raise Exception(f"Invalid robot direction: {self.DIRECTION}")

    def _get_coordinates_of_tile_on_the_left(self) -> models.MapNode:
        if self.DIRECTION == models.ROBOT_DIRECTION.UP:
            return models.MapNode(self.ROW, self.COLUMN-1)
        elif self.DIRECTION == models.ROBOT_DIRECTION.RIGHT:
            return models.MapNode(self.ROW-1, self.COLUMN)
        elif self.DIRECTION == models.ROBOT_DIRECTION.DOWN:
            return models.MapNode(self.ROW, self.COLUMN+1)
        elif self.DIRECTION == models.ROBOT_DIRECTION.LEFT:
            return models.MapNode(self.ROW+1, self.COLUMN)
        else:
            raise Exception(f"Invalid robot direction: {self.DIRECTION}")

    def _get_surrounding_tiles_up(self) -> models.SurroundingTiles:
        forward = self.MAP[self.ROW-1, self.COLUMN]
        right = self.MAP[self.ROW, self.COLUMN+1]
        back = self.MAP[self.ROW+1, self.COLUMN]
        left = self.MAP[self.ROW, self.COLUMN-1]

        return models.SurroundingTiles(left, forward, right, back)

    def _get_surrounding_tiles_right(self) -> models.SurroundingTiles:
        forward = self.MAP[self.ROW, self.COLUMN+1]
        right = self.MAP[self.ROW+1, self.COLUMN]
        back = self.MAP[self.ROW, self.COLUMN-1]
        left = self.MAP[self.ROW-1, self.COLUMN]

        return models.SurroundingTiles(left, forward, right, back)

    def _get_surrounding_tiles_down(self) -> models.SurroundingTiles:
        forward = self.MAP[self.ROW+1, self.COLUMN]
        right = self.MAP[self.ROW, self.COLUMN-1]
        back = self.MAP[self.ROW-1, self.COLUMN]
        left = self.MAP[self.ROW, self.COLUMN+1]

        return models.SurroundingTiles(left, forward, right, back)

    def _get_surrounding_tiles_left(self) -> models.SurroundingTiles:
        forward = self.MAP[self.ROW, self.COLUMN-1]
        right = self.MAP[self.ROW-1, self.COLUMN]
        back = self.MAP[self.ROW, self.COLUMN+1]
        left = self.MAP[self.ROW+1, self.COLUMN]

        return models.SurroundingTiles(left, forward, right, back)

    def _get_surrounding_tiles(self) -> models.SurroundingTiles:
        if self.DIRECTION == models.ROBOT_DIRECTION.UP:
            return self._get_surrounding_tiles_up()
        elif self.DIRECTION == models.ROBOT_DIRECTION.RIGHT:
            return self._get_surrounding_tiles_right()
        elif self.DIRECTION == models.ROBOT_DIRECTION.DOWN:
            return self._get_surrounding_tiles_down()
        elif self.DIRECTION == models.ROBOT_DIRECTION.LEFT:
            return self._get_surrounding_tiles_left()
        else:
            raise Exception(f"Invalid robot direction: {self.DIRECTION}")

    def _get_node_neighbours(self, row, column) -> List[models.MapNode]:
        neighbours : List[models.MapNode] = []

        map_rows, map_columns = self.MAP.shape

        is_tile_valid = models.TILE_TYPE.is_passable

        # top
        if row-1 > 0:
            tile = self.MAP[row-1, column]
            if is_tile_valid(tile):
                neighbours.append(models.MapNode(row-1, column))

        # right
        if column+1 < map_columns:
            tile = self.MAP[row, column+1]
            if is_tile_valid(tile):
                neighbours.append(models.MapNode(row, column+1))

        # bottom
        if row+1 < map_rows:
            tile = self.MAP[row+1, column]
            if is_tile_valid(tile):
                neighbours.append(models.MapNode(row+1, column))

        # left
        if column-1 > 0:
            tile = self.MAP[row, column-1]
            if is_tile_valid(tile):
                neighbours.append(models.MapNode(row, column-1))

        return neighbours

    ##################################################################
    # Pathfinding
    ##################################################################

    def _get_nav_graph(self, include_unknown : bool = True):
        """
        `include_unknown` controls whether we include in the navigation graph tiles,
        which we know exist, but still haven't explored. The flag is raised when
        we navigate towards the closest unexplored tile, but lowered when navigating
        to the charging station.
        """
        G = nx.Graph()

        is_tile_valid = models.TILE_TYPE.is_passable if include_unknown else models.TILE_TYPE.is_passable_and_explored

        edges : Dict[models.MapNode, List[models.MapNode]] = {}

        # create nodes
        it = np.nditer(self.MAP, flags=['multi_index'])
        while not it.finished:
            row, column = it.multi_index
            tile = self.MAP[row, column]
            if is_tile_valid(tile):
                node = models.MapNode(row, column)
                G.add_node(node)
                edges[node] = self._get_node_neighbours(row, column)
            it.iternext()

        # create edges
        for k, v in edges.items():
            for edge in v:
                G.add_edge(k, edge, weight=1)

        return G

    def manhattan_distance(a : models.MapNode, b : models.MapNode) -> float:
        x1, y1 = a.row, a.column
        x2, y2 = b.row, b.column
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def _get_path(self, destination : models.MapNode, source : models.MapNode = None, include_unknown : bool = True):
        """
        Returns a sequence of tiles that represent an eligible path between source and destination.

        `include_unknown` controls whether we include in the navigation graph tiles,
        which we know exist, but still haven't explored. The flag is raised when
        we navigate towards the closest unexplored tile, but lowered when navigating
        to the charging station.

        `source` indicates the starting point for our path plan. By default the
        current robot position is used. However, when evaluating unexplored tiles
        the parameter is set to the coordinates of the evaluated unexplored tile.
        """
        self.GRAPH = self._get_nav_graph(include_unknown)
        origin = source if source else models.MapNode(self.ROW, self.COLUMN)

        if self.TURN_AWARE:
            return algo.astar_path(
                self.GRAPH,
                origin,
                destination,
                self.DIRECTION,
                heuristic=MapExplorer.manhattan_distance,
                weight="weight"
            )
        else:
            return nx.astar_path(
                self.GRAPH,
                origin,
                destination,
                heuristic=MapExplorer.manhattan_distance,
                weight="weight"
            )

    def _get_moves(self, steps : List[Tuple[int,int]]) -> List[RobotMoveType]:
        """
        Takes a sequence of tiles representing a path on the map and calculates
        all necessary robot moves that are required to traverse that path.
        """
        moves : List[models.ROBOT_MOVE_TYPE] = []

        dir = self.DIRECTION
        pos = models.MapNode(self.ROW, self.COLUMN)

        # skip the first step, as it is our current position
        for s in steps[1:]:
            next_pos = models.MapNode(s.row, s.column)
            row_delta = next_pos.row - pos.row
            col_delta = next_pos.column - pos.column

            step_moves = []

            if row_delta > 0:
                # rotate down and move forward
                step_moves.extend(REORIENTATION_MAP[dir][models.ROBOT_DIRECTION.DOWN])
                step_moves.append(models.ROBOT_MOVE_TYPE.FORWARD)

                # TODO: not ideal to set it here, but will do for now
                pos = models.MapNode(pos.row+1, pos.column)
            elif row_delta < 0:
                # rotate up and move forward
                step_moves.extend(REORIENTATION_MAP[dir][models.ROBOT_DIRECTION.UP])
                step_moves.append(models.ROBOT_MOVE_TYPE.FORWARD)

                # TODO: not ideal to set it here, but will do for now
                pos = models.MapNode(pos.row-1, pos.column)
            
            # process the steps so far in order to
            # keep track of the robot direction
            # this is important for cases when we
            # need to move diagonally
            for m in step_moves:
                # store the move
                moves.append(m)
                # update direction after move
                dir = ROTATION_MAP[dir][m]

            step_moves = []

            if col_delta > 0:
                # rotate right and move forward
                step_moves.extend(REORIENTATION_MAP[dir][models.ROBOT_DIRECTION.RIGHT])
                step_moves.append(models.ROBOT_MOVE_TYPE.FORWARD)

                # TODO: not ideal to set it here, but will do for now
                pos = models.MapNode(pos.row, pos.column+1)
            elif col_delta < 0:
                # rotate left and move forward
                step_moves.extend(REORIENTATION_MAP[dir][models.ROBOT_DIRECTION.LEFT])
                step_moves.append(models.ROBOT_MOVE_TYPE.FORWARD)

                # TODO: not ideal to set it here, but will do for now
                pos = models.MapNode(pos.row, pos.column-1)
            
            for m in step_moves:
                # store the move
                moves.append(m)
                # update direction after move
                dir = ROTATION_MAP[dir][m]

        # Convert result to robo_cleaner interfaces type.
        # Usage of own type is remnant of robo miner solution,
        # that is used accross the code.
        # TODO: to be refactored once we have a working solution
        return [RobotMoveType(move_type=m) for m in moves]

    ##################################################################
    # Mapping
    ##################################################################

    def _update_position(self) -> None:
        """
        Updates current robot coordinates after a successful movement
        to a new tile, considering the direction of the movement.
        """
        if self.DIRECTION == models.ROBOT_DIRECTION.DOWN:
            self.ROW += 1
        elif self.DIRECTION == models.ROBOT_DIRECTION.UP:
            self.ROW -= 1
        elif self.DIRECTION == models.ROBOT_DIRECTION.LEFT:
            self.COLUMN -= 1
        elif self.DIRECTION == models.ROBOT_DIRECTION.RIGHT:
            self.COLUMN += 1

    def _handle_row_prepend(self):
        """
        Shift current coordinates when map is resized
        """
        self.ROW += 1
        self.CHARGER_LOCATION.row += 1

    def _handle_column_prepend(self):
        """
        Shift current coordinates when map is resized
        """
        self.COLUMN += 1
        self.CHARGER_LOCATION.column += 1

    def _get_unexplored_count(self):
        """
        Counts the total number of unexplored and uncleaned tiles, that
        we are aware of (based on currently explored tiles
        and knowledge that the map is a rectangle).
        """
        unique, counts = np.unique(self.MAP, return_counts=True)
        values_map = dict(zip(unique, counts))

        return values_map.get(models.TILE_TYPE.UNKNOWN, 0) + \
            values_map.get(models.TILE_TYPE.DIRT_1, 0) + \
                values_map.get(models.TILE_TYPE.DIRT_2, 0) + \
                    values_map.get(models.TILE_TYPE.DIRT_3, 0)

    def _update_map(self):
        """
        Resizes our representation of the game map dynamically
        as the robot explores it. All relative coordinates we
        keep as we go are shifted with each resizing of the map.
        """
        map_rows, map_columns = self.MAP.shape

        if self.ROW == 0:
            # we're at the top corner of the explored map
            # so we prepend a row
            self.MAP = np.pad(self.MAP, ((1,0),(0,0)))
            # update current row index
            self._handle_row_prepend()

        if self.COLUMN == 0:
            # we're at the left corner of the explored map
            # so we prepend a column
            self.MAP = np.pad(self.MAP, ((0,0),(1,0)))
            # update current column index
            self._handle_column_prepend()

        if self.COLUMN+1 == map_columns:
            # we're at the right corner of the explored map
            # so we append a column
            self.MAP = np.pad(self.MAP, ((0,0),(0,1)))

        if self.ROW+1 == map_rows:
            # we're at the bottom corner of the explored map
            # so we append a row
            self.MAP = np.pad(self.MAP, ((0,1),(0,0)))

        self.UNEXPLORED_TILES = self._get_unexplored_count()

    def _mark_map_border(self, tile_ahead : models.MapNode):
        """
        Sets all border tiles to OUT_OF_BOUND type, once we hit one such tile.
        This is a speculative move, based on the knowledge that the map is
        rectangular. This preemptively fills some of the internal map state
        we keep.
        """
        rows, columns = self.MAP.shape
        # determine which rectangle side we're at
        if tile_ahead.row == 0:
            # top
            self._ensure_continuous_border(models.ROBOT_DIRECTION.UP)
            # ensure side borders are filled
            self._ensure_continuous_border(models.ROBOT_DIRECTION.LEFT, ensure_exists=True)
            self._ensure_continuous_border(models.ROBOT_DIRECTION.RIGHT, ensure_exists=True)
        elif tile_ahead.column == 0:
            # left
            self._ensure_continuous_border(models.ROBOT_DIRECTION.LEFT)
            # ensure side borders are filled
            self._ensure_continuous_border(models.ROBOT_DIRECTION.UP, ensure_exists=True)
            self._ensure_continuous_border(models.ROBOT_DIRECTION.DOWN, ensure_exists=True)
        elif tile_ahead.row == rows-1:
            # bottom
            self._ensure_continuous_border(models.ROBOT_DIRECTION.DOWN)
            # ensure side borders are filled
            self._ensure_continuous_border(models.ROBOT_DIRECTION.LEFT, ensure_exists=True)
            self._ensure_continuous_border(models.ROBOT_DIRECTION.RIGHT, ensure_exists=True)
        elif tile_ahead.column == columns-1:
            # right
            self._ensure_continuous_border(models.ROBOT_DIRECTION.RIGHT)
            # ensure side borders are filled
            self._ensure_continuous_border(models.ROBOT_DIRECTION.UP, ensure_exists=True)
            self._ensure_continuous_border(models.ROBOT_DIRECTION.DOWN, ensure_exists=True)

    def _ensure_continuous_border(self, dir : models.ROBOT_DIRECTION, ensure_exists : bool = False):
        """
        A companion function to `_mark_map_border`.

        `ensure_exists` denotes whether we are processing a border in front of us, or to one of
        our sides. The difference is that for a border in front of us it is safe to set all
        tiles to the OUT_OF_BOUND value. However, for borders to our sides we need to be more
        careful, since side borders share tile with other borders, which may or may not be
        discovered yet. E.g. ('U': unexplored, 'O': out of bound, 'X': robot position)

        O U U U U
        O U U X O
        O U U U U
        O U U U U

        In the example above if the robot is facing to the right and we found an OUT_OF_BOUND tile,
        it is safe to set all tiles in the last column to 'O'. However, when checking the top and
        bottom borders (which are respectively to our left and right), we should discard the first
        and last tile in each respective border, if we want to infer whether that border has already
        been hit. Otherwise, is we blindly test all border tiles for an 'O' value, we would wrongly
        assume both borders ahve been hit by the robot.
        """
        rows, columns = self.MAP.shape
        converted = 0

        if dir == models.ROBOT_DIRECTION.UP:
            # top
            border = self.MAP[0]
            # we should ignore the first and last tiles, as they
            # are shared with another rectangle/map side, which
            # may be set to OUT_OF_BOUNDS
            stripped_border = border[1:-1]
            border_exists = len(stripped_border[stripped_border == models.TILE_TYPE.OUT_OF_BOUND]) > 0 if ensure_exists else True
            if border_exists:
                self.MAP[0] = [models.TILE_TYPE.OUT_OF_BOUND for i in range(columns)]
        elif dir == models.ROBOT_DIRECTION.LEFT:
            # left
            border = self.MAP[:, 0]
            # we should ignore the first and last tiles, as they
            # are shared with another rectangle/map side, which
            # may be set to OUT_OF_BOUNDS
            stripped_border = border[1:-1]
            border_exists = len(stripped_border[stripped_border == models.TILE_TYPE.OUT_OF_BOUND]) > 0 if ensure_exists else True
            if border_exists:
                self.MAP[:, 0] = [models.TILE_TYPE.OUT_OF_BOUND for i in range(rows)]
        elif dir == models.ROBOT_DIRECTION.DOWN:
            # bottom
            border = self.MAP[rows-1]
            # we should ignore the first and last tiles, as they
            # are shared with another rectangle/map side, which
            # may be set to OUT_OF_BOUNDS
            stripped_border = border[1:-1]
            border_exists = len(stripped_border[stripped_border == models.TILE_TYPE.OUT_OF_BOUND]) > 0 if ensure_exists else True
            if border_exists:
                self.MAP[rows-1] = [models.TILE_TYPE.OUT_OF_BOUND for i in range(columns)]
        elif dir == models.ROBOT_DIRECTION.RIGHT:
            # right
            border = self.MAP[:, columns-1]
            # we should ignore the first and last tiles, as they
            # are shared with another rectangle/map side, which
            # may be set to OUT_OF_BOUNDS
            stripped_border = border[1:-1]
            border_exists = len(stripped_border[stripped_border == models.TILE_TYPE.OUT_OF_BOUND]) > 0 if ensure_exists else True
            if border_exists:
                self.MAP[:, columns-1] = [models.TILE_TYPE.OUT_OF_BOUND for i in range(rows)]

    ##################################################################
    # Navigation
    ##################################################################

    def _detect_unexplored(self):
        """
        Checks all immediate neighbours, looking for at least one
        that is both passable (not an obstacle or out of bound)
        and is unexplored. This is used when deciding if we should
        continue moving in a simple DFS manner, or navigate
        directly to the closest unexplored tile.
        """
        for n in self._get_node_neighbours(self.ROW, self.COLUMN):
            val = self.MAP[n.row, n.column]
            if models.TILE_TYPE.is_passable_and_unexplored(val):
                return True
        return False

    def _explore_neighbours(self) -> models.MapUpdateResult:
        """
        Picks (DFS-style) a neighbouring tile to explore, if one is eligible.
        Otherwise backtracks (or bails out if we explored the entire map).
        """
        check_tile = models.TILE_TYPE.is_passable_and_unexplored

        if check_tile(self.SURROUNDING_TILES.forward):
            return models.MapUpdateResult(
                next_step=models.MapMoveResult.CONTINUE,
                move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.FORWARD))
        elif check_tile(self.SURROUNDING_TILES.right):
            return models.MapUpdateResult(
                next_step=models.MapMoveResult.CONTINUE,
                move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.ROTATE_RIGHT))
        elif check_tile(self.SURROUNDING_TILES.left):
            return models.MapUpdateResult(
                next_step=models.MapMoveResult.CONTINUE,
                move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.ROTATE_LEFT))
        else:
            if self.UNEXPLORED_TILES > 0:
                return models.MapUpdateResult(
                    next_step=models.MapMoveResult.BACKTRACK,
                    move_type=models.ROBOT_MOVE_TYPE.FORWARD)
            else:
                return models.MapUpdateResult(
                    next_step=models.MapMoveResult.FINISH,
                    move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.FORWARD)
                )

    def _find_closest_unexplored_tile(self, debug : Optional[bool] = None, blacklist : Optional[List[models.MapNode]] = None) -> Optional[models.MapNode]:
        """
        If there are no eligible tiles in our immediate neighbours, we use a heuristic to
        find the closest eligible tile in our viccinity.
        """
        if debug:
            print("Finding closest unexplored tile...")

        initial_row = self.ROW
        initial_column = self.COLUMN
        max_row, max_column = self.MAP.shape
        explored_queue = set((initial_row, initial_column))
        total_considered = 0

        # check tiles in an eccentrically expanding spiral
        for row, column in algo.GenSpiral(initial_row, initial_column):
            total_considered += 1
            if debug:
                print(f"Checking ({row}, {column})")

            already_checked = (row, column) in explored_queue
            row_invalid = row < 0 or row >= max_row
            column_invalid = column < 0 or column >= max_column
            
            # stop condition
            if abs(row - initial_row) >= max_row and abs(column - initial_column) >= max_column:
                if debug:
                    print(f"Hit stop condition")
                    print(f"row: {row}, column: {column}")
                break
            
            # prune out of bounds and already checked coordinates
            if row_invalid or column_invalid or already_checked:
                if debug:
                    print(f"Pruning [row_invalid: {row_invalid}, column_invalid: {column_invalid}, already_checked: {already_checked}]")
                continue
            
            explored_queue.add((row, column))
            val = self.MAP[row, column]
            if models.TILE_TYPE.is_unexplored(val):
                if debug:
                    print(f"Bingo! [{row}, {column}]. Total considered: {total_considered}")

                result = models.MapNode(row, column)

                if blacklist and result in blacklist:
                    continue
                else:
                    return result
            elif debug:
                print(f"Not passable and unexplored [passable: {models.TILE_TYPE.is_passable(val)}, unexplored: {models.TILE_TYPE.is_unexplored(val)}]")
        
        if debug:
            print(f"Bailing out! Total considered: {total_considered}")
        return None

    def _advanced_find_closest_unexplored_tile(self) -> Optional[models.MapNode]:
        """
        A smarter version of `_find_closest_unexplored_tile`, which picks the 3
        closest candidate tiles, based on the quick but not entirely accurate
        manhattan distance. Then for each candidate calculates the actual
        turn-based distance and picks the actually closest one. This allows us to
        avoid cases in which a nearby obstacle makes an otherwise close tile
        be practically far away, due to the need to go around the obstacle.
        """
        N = 3
        results : List[models.MapNode] = []
        unreachable : List[models.MapNode] = []

        # get up to N closest unexplored tiles
        while (self.UNEXPLORED_TILES > 0 and len(results) < N):
            x = self._find_closest_unexplored_tile(blacklist=results+unreachable)
            if x:
                # ensure x is reachable
                try:
                    self._get_path(x)
                except nx.exception.NetworkXNoPath:
                    print(f"Discarding {x} due to unreachability from {self.ROW}, {self.COLUMN}")
                    unreachable.append(x)
                    continue
                results.append(x)
            else:
                break

        # calculate their distance in moves
        costs = {}
        for i, r in enumerate(results):
            steps = self._get_path(r)
            costs[len(self._get_moves(steps))] = i       

        # pick the one cheapest in terms of moves
        return results[costs[min(costs.keys())]] if results else None

    def _return_to_charger(self):
        """
        Navigates the robot back to the charging station for some sweet juice time.
        """
        if self.DEBUG:
            print("Return to charger")

        res = self.navigate(self.CHARGER_LOCATION.asMapNode(), include_unknown=False)

        if self.DEBUG:
            print("Reached charger")

        self._charge()

        if self.DEBUG:
            print("Charged")

        return res

    def _backtrack(self) -> models.MapUpdateResult:
        """
        Gets the robots out of unpleasant situations. Sort of a best friend ;)
        """
        self.SURROUNDING_TILES = self._get_surrounding_tiles()
        check_tile = models.TILE_TYPE.is_passable

        # Special case: we are in an reentrant dirt tile
        # and all our neighbours are explored. This special treatment is
        # necessary, because without it the robot gets stuck.
        if models.TILE_TYPE.is_reentrant_dirt(self.MAP[self.ROW, self.COLUMN]) and not self._detect_unexplored():
            current_node = models.MapNode(self.ROW, self.COLUMN)

            if check_tile(self.SURROUNDING_TILES.left):
                self.navigate(self._get_coordinates_of_tile_on_the_left())
                return self.navigate(current_node)
            elif check_tile(self.SURROUNDING_TILES.right):
                self.navigate(self._get_coordinates_of_tile_on_the_right())
                return self.navigate(current_node)
            elif check_tile(self.SURROUNDING_TILES.back):
                self.navigate(self._get_coordinates_of_tile_behind())
                return self.navigate(current_node)
            elif check_tile(self.SURROUNDING_TILES.forward):
                self.navigate(self._get_coordinates_of_tile_ahead())
                return self.navigate(current_node)

        # Regular process
        if check_tile(self.SURROUNDING_TILES.right):
            return models.MapUpdateResult(
                next_step=models.MapMoveResult.CONTINUE,
                move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.ROTATE_RIGHT))
        elif check_tile(self.SURROUNDING_TILES.left):
            return models.MapUpdateResult(
                next_step=models.MapMoveResult.CONTINUE,
                move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.ROTATE_LEFT))
        else:
            move_type = RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.ROTATE_RIGHT)
            res = self.move(move_type)
            return self.move(move_type)

    ##################################################################
    # Battery
    ##################################################################

    def _get_moves_to_charger(self, source : models.MapNode = None):
        """
        Calculates the number of robot moves needed to reach the charging station.
        """
        include_unknown = False
        if source:
            # if we are checking a tile yet to be explored
            if self.MAP[source.row, source.column] == models.TILE_TYPE.UNKNOWN:
                include_unknown = True

        steps = self._get_path(self.CHARGER_LOCATION.asMapNode(), source=source, include_unknown=include_unknown)
        return self._get_moves(steps)

    def _charge_necessary(self, source : models.MapNode = None):
        """
        Calculates if the robots needs to go home for some rejuvenation.
        """
        moves_to_charger = self._get_moves_to_charger(source)
        return (len(moves_to_charger)+2) >= self.BATTERY.moves_left

    def _charge(self, turns : int = 0):
        res = self.charge_battery_client.charge(turns)
        self.MOVES += res.turns_spend_charging
        self.BATTERY.update(res.battery_status)
        self.print_state()

    def _update_battery_status(self):
        status = self.battery_status_client.query()
        self.BATTERY.update(status)

    ##################################################################
    # Commands
    ##################################################################

    def move(self, m : RobotMoveType) -> models.MapUpdateResult:
        # sleep between moves, trying to reduce the number of
        # random `robo_cleaner_gui` crashes.
        if self.RETARDATION > 0:
            time.sleep(self.RETARDATION)

        self.LAST_MOVE_TYPE = m.move_type
        self.robot_move_client.move(m)

        # Break navigation sequence if we encountered an obstacle
        if self.LAST_MOVE_CANCELLED:
            self.LAST_MOVE_CANCELLED = False

            if self.NAVIGATING:
                return models.MapUpdateResult(
                    next_step=models.MapMoveResult.CANCEL,
                    move_type=models.ROBOT_MOVE_TYPE.FORWARD
                )

        if self.DEBUG:
            self.print_state()
        
        if not self.NAVIGATING:
            if self._charge_necessary():
                return self._return_to_charger()
            elif self._detect_unexplored():
                # there are unexplored tiles in our immediate neighbours
                return self._explore_neighbours()
            else:
                # find closest unexplored tile
                closest_unexplored = self._advanced_find_closest_unexplored_tile()
                if closest_unexplored:
                    # navigate to it
                    return self.navigate(closest_unexplored)
                else:
                    # we have explored the entire map
                    return self._explore_neighbours()
        else:
            # skip calculations if we are navigating and thus not
            # interested in figuring out our next move
            return models.MapUpdateResult(
                next_step=models.MapMoveResult.CONTINUE,
                move_type=models.ROBOT_MOVE_TYPE.FORWARD
            )

    def authenticate(self, user : str, repo : str, commit : str):
        self.authenticator.authenticate(user, repo, commit)
        print("Authentication complete")
    
    def reveal_map(self):
        res = self.init()
        print("Revealing map...")

        while res.next_step != models.MapMoveResult.FINISH:
            if res.next_step == models.MapMoveResult.BACKTRACK:
                res = self._backtrack()
            else:
                res = self.move(res.move_type)

                # Opportunistic optimization: if we happen to pass via the charger while
                # exploring the map, why not take a sip?
                if models.MapNode(self.ROW, self.COLUMN) == self.CHARGER_LOCATION.asMapNode():
                    if self.BATTERY.moves_left < (self.BATTERY.max_moves_on_full_energy-3):
                        self._charge()
                        res = models.MapUpdateResult(
                                next_step=models.MapMoveResult.CONTINUE,
                                move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.ROTATE_RIGHT))

        print(f"Revealed map in {self.MOVES} moves.")
        input("Human, write the number of moves down, please!")
        self._return_to_charger()

    def navigate(self, destination, include_unknown : bool = True):
        not_returning_to_charger = self.CHARGER_LOCATION.asMapNode() != destination

        # Ensure this is not our last voyage into the unknown
        if not_returning_to_charger and self._charge_necessary(source=destination):
            self._return_to_charger()

        if self.DEBUG:
            print("Starting navigation...")

        self.NAVIGATING = True
        self.NAVIGATION_TARGET = destination

        steps = self._get_path(destination, include_unknown=include_unknown)
        moves = self._get_moves(steps)

        for m in moves:
            res = self.move(m)
            if res.next_step == models.MapMoveResult.CANCEL:
                # if we hit an obstacle while navigating, bail out
                break

        result = self._explore_neighbours()

        self.NAVIGATING = False
        self.NAVIGATION_TARGET = None

        if self.DEBUG:
            print("Finished navigation.")

        return result

        