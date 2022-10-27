from robo_miner_controller_py import models, service_clients, algo
import numpy as np
import networkx as nx
from typing import (
    List,
    Dict,
    Tuple,
    Optional
)
from pprint import pprint
import math
import sys

# Credits: https://math.stackexchange.com/a/3448361
def GenSpiral(x, y):
    for n in range(sys.maxsize):
        k = math.ceil((math.sqrt(n) - 1) / 2.0)
        t = 2 * k + 1
        m = t ** 2
        t = t - 1
        if n >= m - t:
            yield x + k - (m - n), y - k
        else:
            m = m - t
        if n >= m - t:
            yield x + -k, y -k + (m - n)
        else:
            m = m - t
        if n >= m - t:
            yield x -k + (m - n), y + k
        else:
            yield x + k, y + k - (m - n - t)

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
        # TODO: remove?
        self.INITIAL_TILE : models.TILE_TYPE = None
        self.DIRECTION : models.ROBOT_DIRECTION = None
        self.SURROUNDING_TILES : models.SurroundingTiles = None
        self.ROW = 0
        self.COLUMN = 0
        self.MAP = np.zeros((1,1), dtype=int)
        self.GRAPH = nx.Graph()
        self.UNEXPLORED_TILES = 0
        self.DEBUG = debug
        self.NAVIGATING = False
        self.TURN_AWARE = use_turn_aware_pathfinding
        self.initial_position_client = service_clients.QueryInitialRobotPositionClientAsync(debug)
        self.move_client = service_clients.RobotMoveClientAsync(debug)
        self.validate_client = service_clients.FieldMapValidateClientAsync(debug)
    
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
        self.INITIAL_TILE = models.TILE_TYPE(response.robot_initial_tile)
        self.MAP[0,0] = self.INITIAL_TILE.value + models.EXPLORED_MARKER

        return self.update(models.ROBOT_MOVE_TYPE.UNKNOWN, response.robot_position_response)

    def _get_node_neighbours(self, row, column) -> List[models.MapNode]:
        neighbours : List[models.MapNode] = []

        map_rows, map_columns = self.MAP.shape

        # top
        if row-1 > 0:
            tile = self.MAP[row-1, column]
            if models.TILE_TYPE.is_passable(tile):
                neighbours.append(models.MapNode(row-1, column))

        # right
        if column+1 < map_columns:
            tile = self.MAP[row, column+1]
            if models.TILE_TYPE.is_passable(tile):
                neighbours.append(models.MapNode(row, column+1))

        # bottom
        if row+1 < map_rows:
            tile = self.MAP[row+1, column]
            if models.TILE_TYPE.is_passable(tile):
                neighbours.append(models.MapNode(row+1, column))

        # left
        if column-1 > 0:
            tile = self.MAP[row, column-1]
            if models.TILE_TYPE.is_passable(tile):
                neighbours.append(models.MapNode(row, column-1))

        return neighbours

    def _get_nav_graph(self):
        G = nx.Graph()

        edges : Dict[models.MapNode, List[models.MapNode]] = {}

        # create nodes
        it = np.nditer(self.MAP, flags=['multi_index'])
        while not it.finished:
            row, column = it.multi_index
            tile = self.MAP[row, column]
            if models.TILE_TYPE.is_passable(tile):
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
        map_rows, map_columns = self.MAP.shape

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
        check_tile = models.TILE_TYPE.is_passable_and_unexplored if self._detect_unexplored() else models.TILE_TYPE.is_passable

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

    def _find_closest_unexplored_tile(self) -> Optional[models.MapNode]:
        initial_row = self.ROW
        initial_column = self.COLUMN
        max_row, max_column = self.MAP.shape
        explored_queue = set((initial_row, initial_column))

        # check tiles in a concentrically expanding spiral
        for row, column in GenSpiral(initial_row, initial_column):
            already_checked = (row, column) in explored_queue
            
            # stop condition
            row_invalid = row < 0 or row >= max_row
            column_invalid = column < 0 or column >= max_column
            if row_invalid and column_invalid:
                break
            
            # prune out of bounds and already checked coordinates
            if row_invalid or column_invalid or already_checked:
                continue
            
            explored_queue.add((row, column))
            val = self.MAP[row, column]
            if models.TILE_TYPE.is_passable_and_unexplored(val):
                return models.MapNode(row, column)
        
        return None

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
            if self._detect_unexplored():
                # there are unexplored tiles in our immediate neighbours
                return self._explore_neighbours()
            else:
                # find closest unexplored tile
                closest_unexplored = self._find_closest_unexplored_tile()
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
    
    def reveal_map(self):
        res = self.init()

        while res.next_step != models.MapMoveResult.FINISH:
            if res.next_step == models.MapMoveResult.BACKTRACK:
                # Turn around
                move_response = self.move_client.move(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT)
                res = self.update(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT, move_response)
                move_response = self.move_client.move(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT)
                res = self.update(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT, move_response)
            else:
                move_response = self.move_client.move(res.move_type)
                res = self.update(res.move_type, move_response)

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

        