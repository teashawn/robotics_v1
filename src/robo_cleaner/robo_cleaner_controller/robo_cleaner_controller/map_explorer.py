from logging import error
from robo_cleaner_controller import models, service_clients, algo
from robo_cleaner_interfaces.msg import RobotMoveType
import numpy as np
import networkx as nx
from typing import (
    List,
    Dict,
    Set,
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
    def __init__(self, debug : bool, use_turn_aware_pathfinding: bool):
        self.DIRECTION : models.ROBOT_DIRECTION = None
        self.ROW = 1
        self.COLUMN = 1
        self.UNEXPLORED_TILES = 8
        self.MOVES = 0
        self.SURROUNDING_TILES = None
        self.LAST_MOVE_TYPE = None
        self.CHARGER_LOCATION = models.Coordinates(1,1)
        self.BATTERY = models.Battery()
        self.DEBUG = debug
        self.NAVIGATING = False
        self.NAVIGATION_TARGET = None
        self.TURN_AWARE = use_turn_aware_pathfinding

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
            # TODO: handle rejection
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
                #self.UNEXPLORED_TILES -= 1

            if self.DEBUG:
                print(f"Cancelling move {tile_ahead}, because tile ahead is {models.TILE_TYPE(approaching_field_marker)}")

            self.robot_move_client.cancel_move()

    def _move_completed(self, result):
        if result.processed_field_marker == 0:
            # goal was cancelled
            self.SURROUNDING_TILES = self._get_surrounding_tiles()
            self._update_battery_status()
            return

        if self.LAST_MOVE_TYPE == RobotMoveType.FORWARD:
            self._update_position()
            self.MAP[self.ROW, self.COLUMN] = result.processed_field_marker

            # NB This is important to keep straight the count of tiles that need to be visited!!!
            # if models.TILE_TYPE.is_reentrant_dirt(result.processed_field_marker):
            #     self.UNEXPLORED_TILES += 1

            # if not models.TILE_TYPE.is_unexplored(result.processed_field_marker):
            #     self.UNEXPLORED_TILES -= 1

            #self._update_map()

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

        # if self.UNEXPLORED_TILES <= 0:
        #     #input(f"Unexplored: {self.UNEXPLORED_TILES}")
        #     self.UNEXPLORED_TILES += 10

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
        left = self.MAP[self.ROW+1, self.COLUMN]

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
        """
        Add cost for turning, detected by change in either X or Y
        Factor in obstacles by:
            - get minimal rectangle of all tiles inclusing origin and destination
            - estimate cost of avoiding by calculating additional
                moves necessary by calculating X and Y offsets of obstacles
                Use self.MAP for more efficient access to obstacles
                Maybe implement obstacles map extraction
        """
        x1, y1 = a.row, a.column
        x2, y2 = b.row, b.column
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def _get_path(self, destination : models.MapNode, source : models.MapNode = None, include_unknown : bool = True):
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
        unique, counts = np.unique(self.MAP, return_counts=True)
        values_map = dict(zip(unique, counts))

        return values_map.get(models.TILE_TYPE.UNKNOWN, 0) + \
            values_map.get(models.TILE_TYPE.DIRT_1, 0) + \
                values_map.get(models.TILE_TYPE.DIRT_2, 0) + \
                    values_map.get(models.TILE_TYPE.DIRT_3, 0)

    def _update_map(self):
        map_rows, map_columns = self.MAP.shape

        if self.ROW == 0:
            # we're at the top corner of the explored map
            # so we prepend a row
            self.MAP = np.pad(self.MAP, ((1,0),(0,0)))
            # update current row index
            self._handle_row_prepend()
            #self.UNEXPLORED_TILES += map_columns

        if self.COLUMN == 0:
            # we're at the left corner of the explored map
            # so we prepend a column
            self.MAP = np.pad(self.MAP, ((0,0),(1,0)))
            # update current column index
            self._handle_column_prepend()
            #self.UNEXPLORED_TILES += map_rows

        if self.COLUMN+1 == map_columns:
            # we're at the right corner of the explored map
            # so we append a column
            self.MAP = np.pad(self.MAP, ((0,0),(0,1)))
            #self.UNEXPLORED_TILES += map_rows

        if self.ROW+1 == map_rows:
            # we're at the bottom corner of the explored map
            # so we append a row
            self.MAP = np.pad(self.MAP, ((0,1),(0,0)))
            #self.UNEXPLORED_TILES += map_columns

        self.UNEXPLORED_TILES = self._get_unexplored_count()

    def _mark_map_border(self, tile_ahead : models.MapNode):
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
                converted = len(border[border == 0])
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
                converted = len(border[border == 0])
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
                converted = len(border[border == 0])
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
                converted = len(border[border == 0])
                self.MAP[:, columns-1] = [models.TILE_TYPE.OUT_OF_BOUND for i in range(rows)]

        # if converted > 0:
        #     self.UNEXPLORED_TILES -= converted

    ##################################################################
    # Navigation
    ##################################################################

    def _detect_unexplored(self):
        for n in self._get_node_neighbours(self.ROW, self.COLUMN):
            val = self.MAP[n.row, n.column]
            if models.TILE_TYPE.is_passable_and_unexplored(val):
                return True
        return False

    def _explore_neighbours(self) -> models.MapUpdateResult:
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
                # if self.NAVIGATING:
                #     return models.MapUpdateResult(
                #         next_step=models.MapMoveResult.CONTINUE,
                #         move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.FORWARD)
                #     )
                # else:
                #     # find closest unexplored tile
                #     closest_unexplored = self._advanced_find_closest_unexplored_tile()
                #     return self.navigate(closest_unexplored)
            else:
                return models.MapUpdateResult(
                    next_step=models.MapMoveResult.FINISH,
                    move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.FORWARD)
                )

    def _find_closest_unexplored_tile(self, debug : Optional[bool] = None, blacklist : Optional[List[models.MapNode]] = None) -> Optional[models.MapNode]:
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
            if models.TILE_TYPE.is_unexplored(val): #is_passable_and_unexplored
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
        self.SURROUNDING_TILES = self._get_surrounding_tiles()
        check_tile = models.TILE_TYPE.is_passable

        # if check_tile(self.SURROUNDING_TILES.forward):
        #     return models.MapUpdateResult(
        #         next_step=models.MapMoveResult.CONTINUE,
        #         move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.FORWARD))
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
        include_unknown = False
        if source:
            # if we are checking a tile yet to be explored
            if self.MAP[source.row, source.column] == models.TILE_TYPE.UNKNOWN:
                include_unknown = True

        steps = self._get_path(self.CHARGER_LOCATION.asMapNode(), source=source, include_unknown=include_unknown)
        return self._get_moves(steps)

    def _charge_necessary(self, source : models.MapNode = None):
        moves_to_charger = self._get_moves_to_charger(source)
        return len(moves_to_charger) >= self.BATTERY.moves_left

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
        print("entered")
        self.LAST_MOVE_TYPE = m.move_type
        self.robot_move_client.move(m)
        print("after call")

        if self.DEBUG:
            print("move")
            self.print_state()
        
        if not self.NAVIGATING:
            print("not navigating")
            if self._charge_necessary():
                print("charge necessary")
                return self._return_to_charger()
            elif self._detect_unexplored():
                print("detect unexplored")
                # there are unexplored tiles in our immediate neighbours
                return self._explore_neighbours()
            else:
                print("find closest")
                # find closest unexplored tile
                closest_unexplored = self._advanced_find_closest_unexplored_tile()
                if closest_unexplored:
                    print("closest found")
                    # navigate to it
                    return self.navigate(closest_unexplored)
                else:
                    print("no closest")
                    input("No closest, we've explored the etire map")
                    # we have explored the entire map
                    return self._explore_neighbours()
        else:
            print("finally")
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
            # if self._charge_necessary():
            #     if self.NAVIGATION_TARGET != None:
            #         not_returning_to_charger = self.CHARGER_LOCATION.asMapNode() != self.NAVIGATION_TARGET
            #         if not_returning_to_charger:
            #             #time.sleep(1)
            #             print("Before backtrack")
            #             input()
            #             pass
            if res.next_step == models.MapMoveResult.BACKTRACK:
                res = self._backtrack()
            else:
                res = self.move(res.move_type)

                # Opportunistic optimization
                if models.MapNode(self.ROW, self.COLUMN) == self.CHARGER_LOCATION.asMapNode():
                    if self.BATTERY.moves_left < (self.BATTERY.max_moves_on_full_energy-3):
                        self._charge()
                        res = models.MapUpdateResult(
                                next_step=models.MapMoveResult.CONTINUE,
                                move_type=RobotMoveType(move_type=models.ROBOT_MOVE_TYPE.ROTATE_RIGHT))

        print(f"Revealed map in {self.MOVES} moves.")

    def navigate(self, destination, kamikaze=False, include_unknown : bool = True):
        """
        `kamikaze` allows navigation to locations of no return,
        i.e. we won't have enough battery to reach the charger
        from the destination.
        """
        not_returning_to_charger = self.CHARGER_LOCATION.asMapNode() != destination

        if not_returning_to_charger and self._charge_necessary(source=destination):
            if not kamikaze:
                err_msg = f"Refusing to navigate! Not enough juice to reach charger from destination!"
                raise Exception(err_msg)

        if self.DEBUG:
            print("Starting navigation...")

        self.NAVIGATING = True
        self.NAVIGATION_TARGET = destination

        steps = self._get_path(destination, include_unknown=include_unknown)
        moves = self._get_moves(steps)

        if not not_returning_to_charger:
            print(f"Steps to charger: {steps}")
            print(f"Moves to charger: {moves}")

        for m in moves:
            self.move(m)

        result = self._explore_neighbours()

        self.NAVIGATING = False
        self.NAVIGATION_TARGET = None

        if self.DEBUG:
            print("Finished navigation.")

        return result

        