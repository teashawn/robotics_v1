from robo_miner_controller_py import models
from rclpy.node import Node
import rclpy
from std_msgs.msg import UInt8MultiArray

from robo_miner_interfaces.srv import QueryInitialRobotPosition, RobotMove, FieldMapValidate
from robo_miner_interfaces.msg import RobotMoveType

class QueryInitialRobotPositionClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('query_initial_robot_position_client_async')
        self.DEBUG = debug
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

        if self.DEBUG:
            # Log
            success = response.robot_position_response.success
            error_reason = response.robot_position_response.error_reason
            surrounding_tiles = models.SurroundingTiles(*response.robot_position_response.surrounding_tiles)
            robot_dir = models.ROBOT_DIRECTION(response.robot_position_response.robot_dir)
            self.get_logger().info(
                f"\nSuccess: {success},\nError reason: {error_reason},\nSurrounding tiles: {surrounding_tiles},\nDirection: {robot_dir}\n"
            )
        
        return response

class RobotMoveClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('robot_move_client_async')
        self.moves = 0
        self.DEBUG = debug
        self.cli = self.create_client(RobotMove, 'move_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, move_type : models.ROBOT_MOVE_TYPE):
        req = RobotMove.Request(robot_move_type=RobotMoveType(move_type=move_type))
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def move(self, move_type : models.ROBOT_MOVE_TYPE):
        move_response = self.send_request(move_type)

        if self.DEBUG:
            # Log
            success = move_response.robot_position_response.success
            error_reason = move_response.robot_position_response.error_reason
            surrounding_tiles = models.SurroundingTiles(*move_response.robot_position_response.surrounding_tiles)
            robot_dir = models.ROBOT_DIRECTION(move_response.robot_position_response.robot_dir)

            if success:
                self.moves += 1

            self.get_logger().info(
                f"\nSuccess: {success},\nError reason: {error_reason},\nSurrounding tiles: {surrounding_tiles},\nDirection: {robot_dir}\nMoves: {self.moves}"
            )

        return move_response.robot_position_response

class FieldMapValidateClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('field_map_validate_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(FieldMapValidate, 'field_map_validate')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        req = FieldMapValidate.Request(field_map=UInt8MultiArray())
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def validate(self, ):
        move_response = self.send_request()#move_type

        if self.DEBUG:
            # Log
            # success = move_response.robot_position_response.success
            # error_reason = move_response.robot_position_response.error_reason
            # surrounding_tiles = models.SurroundingTiles(*move_response.robot_position_response.surrounding_tiles)
            # robot_dir = models.ROBOT_DIRECTION(move_response.robot_position_response.robot_dir)

            # if success:
            #     self.moves += 1

            # self.get_logger().info(
            #     f"\nSuccess: {success},\nError reason: {error_reason},\nSurrounding tiles: {surrounding_tiles},\nDirection: {robot_dir}\nMoves: {self.moves}"
            # )

        #return move_response.robot_position_response
            pass