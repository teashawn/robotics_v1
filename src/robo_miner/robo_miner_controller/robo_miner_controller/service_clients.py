from robo_miner_controller import models
from rclpy.node import Node
from rclpy.qos import QoSProfile
import rclpy
from array import array

from robo_miner_interfaces.srv import QueryInitialRobotPosition, RobotMove, FieldMapValidate, LongestSequenceValidate, ActivateMiningValidate
from robo_miner_interfaces.msg import RobotMoveType, UInt8MultiArray, FieldPoint, UserAuthenticate

class Authenticator(Node):
    def __init__(self):
        super().__init__('auth_publisher')
        qos = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.SYSTEM_DEFAULT,
            depth=10
        )

        self.publisher = self.create_publisher(UserAuthenticate, 'user_authenticate', qos)

    def authenticate(self, user : str, repo : str, commit : str):
        self.publisher.publish(UserAuthenticate(
            user=user,
            repository=repo,
            commit_sha=commit
        ))
        

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
        success = move_response.robot_position_response.success
        if success:
                self.moves += 1

        if self.DEBUG:
            error_reason = move_response.robot_position_response.error_reason
            surrounding_tiles = models.SurroundingTiles(*move_response.robot_position_response.surrounding_tiles)
            robot_dir = models.ROBOT_DIRECTION(move_response.robot_position_response.robot_dir)

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

    def send_request(self, m):
        req = FieldMapValidate.Request()

        a = UInt8MultiArray()
        a.rows, a.cols = m.shape
        a.data = array('B', m.flatten())

        req.field_map = a
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def validate(self, m):
        resp = self.send_request(m)
        success = resp.success

        if self.DEBUG:
            success = resp.success
            error_reason = resp.error_reason

            self.get_logger().info(
                f"\nSuccess: {success},\nError reason: {error_reason}"
            )

        return success

class LongestSequenceValidateClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('longest_sequence_validate_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(LongestSequenceValidate, 'longest_sequence_validate')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, points):
        req = LongestSequenceValidate.Request()

        req.sequence_points = [FieldPoint(row=p.row,col=p.column) for p in points]
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def validate(self, points):
        resp = self.send_request(points)
        success = resp.success

        if self.DEBUG:
            success = resp.success
            error_reason = resp.error_reason

            self.get_logger().info(
                f"\nSuccess: {success},\nError reason: {error_reason}"
            )

        return success

class ActivateMiningClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('activate_mining_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(ActivateMiningValidate, 'activate_mining_validate')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        req = ActivateMiningValidate.Request()

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def activate(self):
        resp = self.send_request()
        success = resp.success

        if self.DEBUG:
            success = resp.success
            error_reason = resp.error_reason

            self.get_logger().info(
                f"\nSuccess: {success},\nError reason: {error_reason}"
            )

        return success