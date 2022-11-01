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
import rclpy
import os

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
        self.ROW = 1
        self.COLUMN = 1
        self.MAP = np.zeros((3,3), dtype=int)
        self.GRAPH = nx.Graph()
        self.UNEXPLORED_TILES = 8
        self.CHARGER_LOCATION = models.Coordinates(1,1)
        self.MOVES = 0
        self.DEBUG = debug
        self.NAVIGATING = False
        self.TURN_AWARE = use_turn_aware_pathfinding
        self.BATTERY = models.Battery()

        self.cancelled_moves = []
        self.LAST_MOVE_TYPE = None

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

        if self.DEBUG:
            print("__init__")
            self.print_state()

    def __del__(self):
        self.initial_state_client.destroy_node()
        self.authenticator.destroy_node()
        self.battery_status_client.destroy_node()
        self.charge_battery_client.destroy_node()
        self.move_counter_listener.destroy_node()

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
        if msg.goal_id in self.cancelled_moves:
            return

        approaching_field_marker = msg.feedback.approaching_field_marker
        #progress_percent = msg.feedback.progress_percent
        #print(f'Received feedback: {progress_percent}% {approaching_field_marker}')

        if approaching_field_marker > 0 and not models.TILE_TYPE.is_passable(approaching_field_marker):
            tile_ahead = self._get_coordinates_of_tile_ahead()
            self.MAP[tile_ahead.row, tile_ahead.column] = approaching_field_marker
            self.UNEXPLORED_TILES -= 1

            if self.DEBUG:
                print(f"Cancelling move, because tile ahead is {models.TILE_TYPE(approaching_field_marker)}")

            self.cancelled_moves.append(msg.goal_id)
            self.robot_move_client.cancel_move()
            if self.DEBUG:
                print("_move_feedback cancel")

    def _move_completed(self, result):
        if result.processed_field_marker == 0:
            # goal was cancelled
            return

        if self.LAST_MOVE_TYPE == RobotMoveType.FORWARD:
            self._update_position()
            self.MAP[self.ROW, self.COLUMN] = result.processed_field_marker
            self.UNEXPLORED_TILES -= 1
            self._update_map()
            if self.DEBUG:
                print(f"Entered tile: {models.TILE_TYPE(result.processed_field_marker)}")
        
        self.DIRECTION = ROTATION_MAP[self.DIRECTION][self.LAST_MOVE_TYPE]
        self.update_battery_status()
        if self.DEBUG:
            print("_move_completed")
            self.print_state()
        

    def print_state(self):
        print("####################################")
        print(f"# Map Explorer State:")
        print(f"# Row: {self.ROW}, Column: {self.COLUMN}, Direction: {self.DIRECTION}, Unexplored: {self.UNEXPLORED_TILES}, Map size: {self.MAP.shape}, Moves: {self.MOVES}, Navigating: {self.NAVIGATING}")
        pprint(self.MAP)
        print("####################################")

    def init(self): #  -> models.MapUpdateResult
        response = self.initial_state_client.query()
        initial_tile= models.TILE_TYPE(response.initial_robot_state.robot_tile)
        self.DIRECTION = models.ROBOT_DIRECTION(response.initial_robot_state.robot_dir)
        self.MAP[1,1] = initial_tile.value
        if models.TILE_TYPE(initial_tile.value) == models.TILE_TYPE.CHARGER:
            self.CHARGER_LOCATION = models.MapNode(1,1)
        battery_status = response.initial_robot_state.battery_status
        self.BATTERY.update(battery_status)

        if self.DEBUG:
            print("init")
            self.print_state()
        #return self.update(models.ROBOT_MOVE_TYPE.UNKNOWN, response.initial_robot_state)

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

    def _get_nav_graph(self):
        G = nx.Graph()

        is_tile_valid = models.TILE_TYPE.is_passable

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

    def _update_map(self):
        map_rows, map_columns = self.MAP.shape

        if self.ROW == 0:
            # we're at the top corner of the explored map
            # so we prepend a row
            self.MAP = np.pad(self.MAP, ((1,0),(0,0)))
            # update current row index
            self.ROW += 1
            self.UNEXPLORED_TILES += map_columns

        if self.COLUMN == 0:
            # we're at the left corner of the explored map
            # so we prepend a column
            self.MAP = np.pad(self.MAP, ((0,0),(1,0)))
            # update current column index
            self.COLUMN += 1
            self.UNEXPLORED_TILES += map_rows

        if self.COLUMN+1 == map_columns:
            # we're at the right corner of the explored map
            # so we append a column
            self.MAP = np.pad(self.MAP, ((0,0),(0,1)))
            self.UNEXPLORED_TILES += map_rows

        if self.ROW+1 == map_rows:
            # we're at the bottom corner of the explored map
            # so we append a row
            self.MAP = np.pad(self.MAP, ((0,1),(0,0)))
            self.UNEXPLORED_TILES += map_columns

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

    def update(self, move_type : models.ROBOT_MOVE_TYPE, response) -> models.MapUpdateResult:
        pass
        # if move_type == models.ROBOT_MOVE_TYPE.FORWARD:
        #     # we moved to a new tile, as opposed to only changing direction
        #     self._update_position()
        #     if models.TILE_TYPE.is_passable_and_unexplored(self.MAP[self.ROW, self.COLUMN]):
        #         self.MAP[self.ROW, self.COLUMN] += models.EXPLORED_MARKER
        #         self.UNEXPLORED_TILES -= 1
    
        # self.DIRECTION = models.ROBOT_DIRECTION(response.robot_dir)
        # #self.SURROUNDING_TILES = models.SurroundingTiles(*response.surrounding_tiles)

        # #self._update_map()

        # if self.DEBUG:
        #     self.print_state()
        #     print("\n" + chr(176)*70)

        # if not self.NAVIGATING:
        #     if self._detect_unexplored():
        #         # there are unexplored tiles in our immediate neighbours
        #         return self._explore_neighbours()
        #     else:
        #         # find closest unexplored tile
        #         closest_unexplored = self._advanced_find_closest_unexplored_tile()
        #         if closest_unexplored:
        #             # navigate to it
        #             return self.navigate(closest_unexplored)
        #         else:
        #             # we have explored the entire map
        #             return self._explore_neighbours()
        # else:
        #     # skip calculations if we are navigating and thus not
        #     # interested in figuring out our next move
        #     return models.MapUpdateResult(
        #         next_step=models.MapMoveResult.CONTINUE,
        #         move_type=models.ROBOT_MOVE_TYPE.UNKNOWN)
    
    # def move(self, move_type : RobotMoveType) -> models.MapUpdateResult:
    #     move_response = self.move_client.move(move_type)
    #     return self.update(move_type, move_response)

    def move(self, m : RobotMoveType):
        self.LAST_MOVE_TYPE = m.move_type
        self.robot_move_client.move(m)
        if self.DEBUG:
            print("move")
            self.print_state()
        # check return path length
        # if necessary store current location and return to charge

    def move_test(self):
        self.move(RobotMoveType(move_type=RobotMoveType.FORWARD))
        self.move(RobotMoveType(move_type=RobotMoveType.FORWARD))
        self.move(RobotMoveType(move_type=RobotMoveType.ROTATE_RIGHT))
        self.move(RobotMoveType(move_type=RobotMoveType.FORWARD))
        self.move(RobotMoveType(move_type=RobotMoveType.ROTATE_LEFT))
        self.move(RobotMoveType(move_type=RobotMoveType.FORWARD))
        self.move(RobotMoveType(move_type=RobotMoveType.FORWARD))

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

    def update_battery_status(self):
        status = self.battery_status_client.query()
        self.BATTERY.update(status)
        