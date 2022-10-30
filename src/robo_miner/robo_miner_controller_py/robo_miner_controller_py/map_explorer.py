from robo_miner_controller_py import models, service_clients, algo
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

class MapExplorer:
    def __init__(self, debug : bool, use_turn_aware_pathfinding: bool):
        self.DIRECTION : models.ROBOT_DIRECTION = None
        self.SURROUNDING_TILES : models.SurroundingTiles = None
        self.ROW = 0
        self.COLUMN = 0
        self.MAP = np.zeros((1,1), dtype=int)
        self.GRAPH = nx.Graph()
        self.UNEXPLORED_TILES = 0
        self.DEBUG = debug
        self.NAVIGATING = False
        self.MINING = False
        self.TURN_AWARE = use_turn_aware_pathfinding
        self.LONGEST_SEQUENCE = None

        self.initial_position_client = service_clients.QueryInitialRobotPositionClientAsync(debug)
        self.move_client = service_clients.RobotMoveClientAsync(debug)
        self.validate_client = service_clients.FieldMapValidateClientAsync(debug)
        self.longest_sequence_validate_client = service_clients.LongestSequenceValidateClientAsync(debug)
        self.activate_mining_client = service_clients.ActivateMiningClientAsync(debug)
        self.authenticator = service_clients.Authenticator()
    
    def __del__(self):
        self.initial_position_client.destroy_node()
        self.move_client.destroy_node()
        self.validate_client.destroy_node()

    def print_state(self):
        print("####################################")
        print(f"# Map Explorer State:")
        print(f"# Row: {self.ROW}, Column: {self.COLUMN}, Unexplored: {self.UNEXPLORED_TILES}, Map size: {self.MAP.shape}, Navigating: {self.NAVIGATING}")
        pprint(self.MAP)
        print("####################################")

    def init(self) -> models.MapUpdateResult:
        response = self.initial_position_client.query()
        initial_tile= models.TILE_TYPE(response.robot_initial_tile)
        self.MAP[0,0] = initial_tile.value + models.EXPLORED_MARKER

        return self.update(models.ROBOT_MOVE_TYPE.UNKNOWN, response.robot_position_response)

    def _get_node_neighbours(self, row, column) -> List[models.MapNode]:
        neighbours : List[models.MapNode] = []

        map_rows, map_columns = self.MAP.shape

        is_tile_valid = models.TILE_TYPE.is_passable
        if self.MINING:
            t = self._get_mining_target_tile()
            def is_valid(tt):
                return tt == t
            is_tile_valid = is_valid

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

    def _get_mining_target_tile(self):
        t = self.LONGEST_SEQUENCE[0]
        return self.MAP[t.row, t.column]

    def _get_nav_graph(self):
        G = nx.Graph()

        is_tile_valid = models.TILE_TYPE.is_passable
        if self.MINING:
            t = self._get_mining_target_tile()
            def is_valid(tt):
                return tt == t
            is_tile_valid = is_valid

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

    def get_raw_map(self) -> np.ndarray:
        raw = self.MAP.copy()

        for idx in np.ndindex(raw.shape):
            if raw[idx] > models.EXPLORED_MARKER:
                raw[idx] -= models.EXPLORED_MARKER

        return raw

    def _trace_sequence(node : models.MapNode, explored : Set[models.MapNode], map : np.ndarray) -> List[models.MapNode]:
        result = []

        row = node.row
        col = node.column
        marker = map[row,col]

        max_row, max_col = map.shape

        if row-1 >= 0:
            n = models.MapNode(row-1, col)
            val = map[n.row, n.column]
            if val == marker and n not in explored:
                result.append(n)
                explored.add(n)
        if row+1 < max_row:
            n = models.MapNode(row+1,col)
            val = map[n.row, n.column]
            if val == marker and n not in explored:
                result.append(n)
                explored.add(n)
        if col-1 >= 0:
            n = models.MapNode(row, col-1)
            val = map[n.row, n.column]
            if val == marker and n not in explored:
                result.append(n)
                explored.add(n)
        if col+1 < max_col:
            n = models.MapNode(row,col+1)
            val = map[n.row, n.column]
            if val == marker and n not in explored:
                result.append(n)
                explored.add(n)

        children = []
        for n in result:
            children += MapExplorer._trace_sequence(n, explored, map)

        return result + children

    def get_longest_sequence(self) -> List[models.MapNode]:
        raw_map = self.get_raw_map()
        explored = set()
        longest_sequence = []
        longest_sequence_size = 0

        for idx in np.ndindex(raw_map.shape):
            if not models.TILE_TYPE.is_passable(raw_map[idx]):
                continue

            node = models.MapNode(idx[0], idx[1])
            candidate = [node]
            explored.add(node)

            for c in MapExplorer._trace_sequence(node, explored, raw_map):
                explored.add(c)
                candidate.append(c)

            if len(candidate) > longest_sequence_size:
                longest_sequence = candidate
                longest_sequence_size = len(candidate)

        return longest_sequence

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

    def get_path(self, destination : models.MapNode):
        self.GRAPH = self._get_nav_graph()

        origin = models.MapNode(self.ROW, self.COLUMN)
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

    def get_moves(self, steps : List[Tuple[int,int]]) -> List[models.ROBOT_MOVE_TYPE]:
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

        return moves

    def _update_position(self) -> None:
        if self.DIRECTION == models.ROBOT_DIRECTION.DOWN:
            self.ROW += 1
        elif self.DIRECTION == models.ROBOT_DIRECTION.UP:
            self.ROW -= 1
        elif self.DIRECTION == models.ROBOT_DIRECTION.LEFT:
            self.COLUMN -= 1
        elif self.DIRECTION == models.ROBOT_DIRECTION.RIGHT:
            self.COLUMN += 1

    def _handle_direction_up(self):
        map_rows, map_columns = self.MAP.shape

        if self.SURROUNDING_TILES.forward != models.TILE_TYPE.OUT_OF_BOUND:
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
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.forward):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.forward = self.MAP[self.ROW-1, self.COLUMN]
        if self.SURROUNDING_TILES.left != models.TILE_TYPE.OUT_OF_BOUND:
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
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.left):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.left = self.MAP[self.ROW, self.COLUMN-1]
        if self.SURROUNDING_TILES.right != models.TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.COLUMN+1 == map_columns:
                # we're at the right corner of the explored map
                # so we append a column
                self.MAP = np.pad(self.MAP, ((0,0),(0,1)))
            # store tile value
            if self.MAP[self.ROW, self.COLUMN+1] == 0:
                self.MAP[self.ROW, self.COLUMN+1] = self.SURROUNDING_TILES.right.value
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.right):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.right = self.MAP[self.ROW, self.COLUMN+1]

    def _handle_direction_down(self):
        map_rows, map_columns = self.MAP.shape

        if self.SURROUNDING_TILES.forward != models.TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.ROW+1 == map_rows:
                # we're at the bottom corner of the explored map
                # so we append a row
                self.MAP = np.pad(self.MAP, ((0,1),(0,0)))
            # store tile value
            if self.MAP[self.ROW+1, self.COLUMN] == 0:
                self.MAP[self.ROW+1, self.COLUMN] = self.SURROUNDING_TILES.forward.value
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.forward):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.forward = self.MAP[self.ROW+1, self.COLUMN]
        if self.SURROUNDING_TILES.left != models.TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.COLUMN+1 == map_columns:
                # we're at the right corner of the explored map
                # so we append a column
                self.MAP = np.pad(self.MAP, ((0,0),(0,1)))
            # store tile value
            if self.MAP[self.ROW, self.COLUMN+1] == 0:
                self.MAP[self.ROW, self.COLUMN+1] = self.SURROUNDING_TILES.left.value
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.left):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.left = self.MAP[self.ROW, self.COLUMN+1]
        if self.SURROUNDING_TILES.right != models.TILE_TYPE.OUT_OF_BOUND:
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
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.right):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.right = self.MAP[self.ROW, self.COLUMN-1]

    def _handle_direction_left(self):
        map_rows, map_columns = self.MAP.shape

        if self.SURROUNDING_TILES.forward != models.TILE_TYPE.OUT_OF_BOUND:
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
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.forward):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.forward = self.MAP[self.ROW, self.COLUMN-1]
        if self.SURROUNDING_TILES.left != models.TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.ROW+1 == map_rows:
                # we're at the bottom corner of the explored map
                # so we append a row
                self.MAP = np.pad(self.MAP, ((0,1),(0,0)))
            # store tile value
            if self.MAP[self.ROW+1, self.COLUMN] == 0:
                self.MAP[self.ROW+1, self.COLUMN] = self.SURROUNDING_TILES.left.value
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.left):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.left = self.MAP[self.ROW+1, self.COLUMN]
        if self.SURROUNDING_TILES.right != models.TILE_TYPE.OUT_OF_BOUND:
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
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.right):
                    self.UNEXPLORED_TILES += 1
            else:
                # update surrounding tiles from own map, if available
                self.SURROUNDING_TILES.right = self.MAP[self.ROW-1, self.COLUMN]

    def _handle_direction_right(self):
        map_rows, map_columns = self.MAP.shape

        if self.SURROUNDING_TILES.forward != models.TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.COLUMN+1 == map_columns:
                # we're at the right corner of the explored map
                # so we append a column
                self.MAP = np.pad(self.MAP, ((0,0),(0,1)))
            # store tile value
            if self.MAP[self.ROW, self.COLUMN+1] == 0:
                self.MAP[self.ROW, self.COLUMN+1] = self.SURROUNDING_TILES.forward.value
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.forward):
                    self.UNEXPLORED_TILES += 1
        if self.SURROUNDING_TILES.left != models.TILE_TYPE.OUT_OF_BOUND:
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
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.left):
                    self.UNEXPLORED_TILES += 1
        if self.SURROUNDING_TILES.right != models.TILE_TYPE.OUT_OF_BOUND:
            # extend the map if necessary
            if self.ROW+1 == map_rows:
                # we're at the bottom corner of the explored map
                # so we append a row
                self.MAP = np.pad(self.MAP, ((0,1),(0,0)))
            # store tile value
            if self.MAP[self.ROW+1, self.COLUMN] == 0:
                self.MAP[self.ROW+1, self.COLUMN] = self.SURROUNDING_TILES.right.value
                if models.TILE_TYPE.is_passable(self.SURROUNDING_TILES.right):
                    self.UNEXPLORED_TILES += 1

    def _update_map(self):
        if self.DIRECTION == models.ROBOT_DIRECTION.UP:
            self._handle_direction_up()
        elif self.DIRECTION == models.ROBOT_DIRECTION.DOWN:
            self._handle_direction_down()
        elif self.DIRECTION == models.ROBOT_DIRECTION.LEFT:
            self._handle_direction_left()
        elif self.DIRECTION == models.ROBOT_DIRECTION.RIGHT:
            self._handle_direction_right()

    def _detect_unexplored(self):
        return models.TILE_TYPE.is_passable_and_unexplored(self.SURROUNDING_TILES.forward) \
            or models.TILE_TYPE.is_passable_and_unexplored(self.SURROUNDING_TILES.left) \
            or models.TILE_TYPE.is_passable_and_unexplored(self.SURROUNDING_TILES.right)

    def _explore_neighbours(self) -> models.MapUpdateResult:
        check_tile = models.TILE_TYPE.is_passable_and_unexplored #if self._detect_unexplored() else models.TILE_TYPE.is_passable

        if check_tile(self.SURROUNDING_TILES.forward):
            return models.MapUpdateResult(
                next_step=models.MapMoveResult.CONTINUE,
                move_type=models.ROBOT_MOVE_TYPE.FORWARD)
        elif check_tile(self.SURROUNDING_TILES.right):
            return models.MapUpdateResult(
                next_step=models.MapMoveResult.CONTINUE,
                move_type=models.ROBOT_MOVE_TYPE.ROTATE_RIGHT)
        elif check_tile(self.SURROUNDING_TILES.left):
            return models.MapUpdateResult(
                next_step=models.MapMoveResult.CONTINUE,
                move_type=models.ROBOT_MOVE_TYPE.ROTATE_LEFT)
        else:
            if self.UNEXPLORED_TILES > 0:
                return models.MapUpdateResult(
                    next_step=models.MapMoveResult.BACKTRACK,
                    move_type=models.ROBOT_MOVE_TYPE.FORWARD)
            else:
                return models.MapUpdateResult(
                    next_step=models.MapMoveResult.FINISH,
                    move_type=models.ROBOT_MOVE_TYPE.UNKNOWN
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
            if models.TILE_TYPE.is_passable_and_unexplored(val):
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

        # get up to N closest unexplored tiles
        while (self.UNEXPLORED_TILES > 0 and len(results) < N):
            x = self._find_closest_unexplored_tile(blacklist=results)
            if x:
                results.append(x)
            else:
                break

        # calculate their distance in moves
        costs = {}
        for i, r in enumerate(results):
            steps = self.get_path(r)
            costs[len(self.get_moves(steps))] = i

        # pick the one cheapest in terms of moves
        return results[costs[min(costs.keys())]] if results else None

    def _get_map_subframe(self, frame_size : models.FrameSize) -> models.MapFrame:
        map_frame_size = models.FrameSize.init_with_tuple(self.MAP.shape)

        # sanity check
        if map_frame_size <= frame_size:
            return models.MapFrame(
                top_left=models.MapNode(0, 0),
                top_right=models.MapNode(0, map_frame_size.width-1),
                bottom_right=models.MapNode(map_frame_size.height-1, map_frame_size.width-1),
                bottom_left=models.MapNode(map_frame_size.height-1, 0)
            )

    def update(self, move_type : models.ROBOT_MOVE_TYPE, response) -> models.MapUpdateResult:
        if response.success and move_type == models.ROBOT_MOVE_TYPE.FORWARD:
            # we moved to a new tile, as opposed to only changing direction
            self._update_position()
            if models.TILE_TYPE.is_passable_and_unexplored(self.MAP[self.ROW, self.COLUMN]):
                self.MAP[self.ROW, self.COLUMN] += models.EXPLORED_MARKER
                self.UNEXPLORED_TILES -= 1
    
        self.DIRECTION = models.ROBOT_DIRECTION(response.robot_dir)
        self.SURROUNDING_TILES = models.SurroundingTiles(*response.surrounding_tiles)

        self._update_map()

        if self.DEBUG:
            self.print_state()
            print("\n" + chr(176)*70)

        if not self.NAVIGATING:
            # TODO: abstract DFS vs nav only and choose via flag
            # closest_unexplored = self._advanced_find_closest_unexplored_tile()
            # if closest_unexplored:
            #     return self.navigate(closest_unexplored)
            # else:
            #     return models.MapUpdateResult(
            #             next_step=models.MapMoveResult.FINISH,
            #             move_type=models.ROBOT_MOVE_TYPE.UNKNOWN
            #         )
            if self._detect_unexplored():
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
                move_type=models.ROBOT_MOVE_TYPE.UNKNOWN)
    
    def move(self, move_type : models.ROBOT_MOVE_TYPE) -> models.MapUpdateResult:
        move_response = self.move_client.move(move_type)
        return self.update(move_type, move_response)

    def turn_around(self) -> models.MapUpdateResult:
        move_type = models.ROBOT_MOVE_TYPE.ROTATE_RIGHT
        res = self.move(move_type)
        return self.move(move_type)

    def authenticate(self, user : str, repo : str, commit : str):
        self.authenticator.authenticate(user, repo, commit)
        print("Authentication complete")
    
    def reveal_map(self):
        res = self.init()
        print("Revealing map...")

        while res.next_step != models.MapMoveResult.FINISH:
            if res.next_step == models.MapMoveResult.BACKTRACK:
                res = self.turn_around()
            else:
                res = self.move(res.move_type)

        print(f"Revealed map in {self.move_client.moves} moves.")

    def navigate(self, destination):
        if self.DEBUG:
            print("Starting navigation...")

        self.NAVIGATING = True

        steps = self.get_path(destination)
        moves = self.get_moves(steps)

        for m in moves:
            move_response = self.move_client.move(m)
            self.update(m, move_response)

        self.NAVIGATING = False

        if self.DEBUG:
            print("Finished navigation.")

        return self._explore_neighbours()

    def validate_map(self):
        resp = self.validate_client.validate(self.get_raw_map())
        print(f"Validating map... {'SUCCESS' if resp else 'FAILURE'}")

    def validate_longest_sequence(self):
        self.LONGEST_SEQUENCE = self.get_longest_sequence()
        resp = self.longest_sequence_validate_client.validate(self.LONGEST_SEQUENCE)
        print(f"Validating longest sequence... {'SUCCESS' if resp else 'FAILURE'}")

    def mine_longest_sequence(self):
        self.navigate(self.LONGEST_SEQUENCE[0])
        self.activate_mining_client.activate()
        self.MINING = True
        print("Starting mining...")
        for n in self.LONGEST_SEQUENCE[1:]:
            self.navigate(n)
        self.MINING = False
        print("Finished mining.")
        