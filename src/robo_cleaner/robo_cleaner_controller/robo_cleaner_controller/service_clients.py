from robo_cleaner_controller import models
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
import rclpy
from typing import Callable
from std_msgs.msg import Empty, Int32

from robo_cleaner_interfaces.action import RobotMove
from robo_cleaner_interfaces.srv import QueryInitialRobotState, ChargeBattery, QueryBatteryStatus
from robo_cleaner_interfaces.msg import UserAuthenticate, BatteryStatus, InitialRobotState, RobotMoveType

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
        
class QueryInitialRobotStateClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('query_initial_robot_state_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(QueryInitialRobotState, 'query_initial_robot_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = QueryInitialRobotState.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def query(self):
        response = self.send_request()

        if self.DEBUG:
            # Log
            success = response.success
            error_reason = response.error_reason
            state = response.initial_robot_state
            robot_dir = models.ROBOT_DIRECTION(state.robot_dir)
            initial_tile = state.robot_tile
            self.get_logger().info(
                f"\nSuccess: {success},\nError reason: {error_reason},\nDirection: {robot_dir},\nInitial tile: {initial_tile}\n"
            )
        return response

class FieldMapRevealedSubscriber(Node):
    def __init__(self, cb):
        super().__init__('field_map_revealed_subscriber')

        qos = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.SYSTEM_DEFAULT,
            depth=10
        )

        self.subscription = self.create_subscription(
            Empty,
            'field_map_revealed',
            cb,
            qos)

class RobotMoveCounterSubscriber(Node):
    def __init__(self,debug : bool, cb):
        super().__init__('robo_move_counter_subscriber')

        self.subscription = self.create_subscription(
            Int32,
            'robot_move_counter',
            cb,
            10)

class BatteryStatusClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('battery_status_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(QueryBatteryStatus, 'query_battery_status')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = QueryBatteryStatus.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def query(self):
        response = self.send_request()

        if self.DEBUG:
            # Log
            max_moves_on_full_energy = response.battery_status.max_moves_on_full_energy
            moves_left = response.battery_status.moves_left
            self.get_logger().info(
                f"\nmax_moves_on_full_energy: {max_moves_on_full_energy},\nmoves_left: {moves_left}\n"
            )
        return response.battery_status

class ChargeBatteryClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('charge_battery_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(ChargeBattery, 'charge_battery')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ChargeBattery.Request()

    def send_request(self, turns : int):
        self.req.charge_turns = turns
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def charge(self, turns : int):
        response = self.send_request(turns)

        if self.DEBUG:
            success = response.success
            error_reason = response.error_reason
            battery_status = response.battery_status
            turns_spend_charging = response.turns_spend_charging
            
            self.get_logger().info(
                f"\nsuccess: {success},\nerror_reason: {error_reason},\nbattery_status: {battery_status},\nturns_spend_charging: {turns_spend_charging}\n"
            )
        return response

class RobotMoveActionClient(Node):
    def __init__(self, debug : bool, cb_ack : Callable, cb_completed : Callable, cb_feedback : Callable):
        super().__init__('robot_move_action_client')
        self.DEBUG = debug
        self._action_client = ActionClient(self, RobotMove, 'move_robot')

        self.cb_ack = cb_ack
        self.cb_completed = cb_completed
        self.cb_feedback = cb_feedback

    def _send_goal(self, move_type : RobotMoveType):
        goal_msg = RobotMove.Goal()
        goal_msg.robot_move_type = move_type

        self._action_client.wait_for_server()

        _send_goal_future =  self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.cb_feedback
        )
        _send_goal_future.add_done_callback(self._move_acknowledged)

        return _send_goal_future

    def _move_acknowledged(self, future):
        goal_handle = future.result()
        self.cb_ack(goal_handle.accepted)

    def _move_completed(self, future):

        result = future.result().result
        self.cb_completed(result)

    def _cancel_done(self, future):
        pass

    def move(self, move_type : RobotMoveType):
        _send_goal_future = self._send_goal(move_type)
        rclpy.spin_until_future_complete(self, _send_goal_future)

        self.goal_handle = _send_goal_future.result()
        self.goal_handle_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.goal_handle_future)
        self._move_completed(self.goal_handle_future)

    def cancel_move(self):
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done)
