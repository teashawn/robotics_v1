from robo_miner_controller_py import models
import numpy as np
import networkx as nx
from typing import (
    List,
    Dict
)
from pprint import pprint

class MapExplorer:
    def __init__(self, debug : bool):
        self.INITIAL_TILE : models.TILE_TYPE = None
        self.DIRECTION : models.ROBOT_DIRECTION = None
        self.SURROUNDING_TILES : models.SurroundingTiles = None
        self.ROW = 0
        self.COLUMN = 0
        self.MAP = np.zeros((1,1), dtype=int)
        self.GRAPH = nx.Graph()
        self.UNEXPLORED_TILES = 0
        self.DEBUG = debug

    def print_state(self):
        print("####################################")
        print(f"# Map Explorer State:")
        print(f"# Row: {self.ROW}, Column: {self.COLUMN}, Unexplored: {self.UNEXPLORED_TILES}, Map size: {self.MAP.shape}")
        pprint(self.MAP)
        print("####################################")

    def init(self, response) -> models.MapUpdateResult:
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
                G.add_edge(k, edge)

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

    def _nav_edge_weight(a : models.MapNode, b : models.MapNode, attributes : dict) -> int:
        return 1

    def get_path(self, destination : models.MapNode):
        self.GRAPH = self._get_nav_graph()
        origin = models.MapNode(self.ROW, self.COLUMN)
        return nx.astar_path(self.GRAPH, origin, destination, heuristic=MapExplorer.manhattan_distance, weight=MapExplorer._nav_edge_weight)

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

    def update(self, move_type : models.ROBOT_MOVE_TYPE, response) -> models.MapUpdateResult:
        if response.success and move_type == models.ROBOT_MOVE_TYPE.FORWARD:
            # we moved to a new tile, as opposed to only changing direction
            self._update_position()
            if models.TILE_TYPE.is_passable_and_unexplored(self.MAP[self.ROW, self.COLUMN]):
                self.MAP[self.ROW, self.COLUMN] += models.EXPLORED_MARKER
                self.UNEXPLORED_TILES -= 1
    
        self.DIRECTION = models.ROBOT_DIRECTION(response.robot_dir)
        # TODO: get surrounding value from explored map, is available
        self.SURROUNDING_TILES = models.SurroundingTiles(*response.surrounding_tiles)

        self._update_map()

        if self.DEBUG:
            self.print_state()
            print("\n" + chr(176)*70)
            # TODO: remove
            #input()

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