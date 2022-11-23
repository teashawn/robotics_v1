#from rclpy.qos import qos_profile_default, qos_profile_sensor_data
#from rclpy.qos import QoSPresetProfiles
# SENSOR_DATA, SERVICES_DEFAULT, SYSTEM_DEFAULT
# https://docs.ros2.org/foxy/api/rclpy/api/qos.html

from urscript_interfaces.srv import UrScript, GetEefAngleAxis
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import QoSProfile
import rclpy
from visualization_msgs.msg import MarkerArray, Marker

from ur_controller import constants, util
import asyncio
import threading

def wrap_urscript(payload : str) -> str:
    return f"{constants.FUNC_HEADER}{'  '}{'  '.join([l.strip() for l in payload.splitlines()])}{constants.FUNC_FOOTER}"

def wrap_gripper_urscript(payload : str) -> str:
    return f"{constants.GRIPPER_HEADER}{'  '}{'  '.join([l.strip() for l in payload.splitlines()])}{constants.FUNC_FOOTER}"

class URScriptClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('urscript_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(
            UrScript,
            'urscript_service',
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = UrScript.Request()

    def send_robot_request(self, payload : str):
        self.req.data = wrap_urscript(payload)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_gripper_request(self, payload : str):
        self.req.data = wrap_gripper_urscript(payload)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
class PowerOnClientAsync(Node):
    def __init__(self, debug : bool):
        super().__init__('power_on_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(
            Trigger,
            'dashboard_client/power_on'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class BrakeReleaseClientAsync(Node):
    def __init__(self, debug : bool):
        super().__init__('brake_release_async')
        self.DEBUG = debug
        self.cli = self.create_client(
            Trigger,
            'dashboard_client/brake_release'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class GetEefAngleAxisClientAsync(Node):
    def __init__(self, debug : bool):
        super().__init__('get_eef_angle_axis_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(
            GetEefAngleAxis,
            'get_eef_angle_axis'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetEefAngleAxis.Request()

    def send_request(self):
        #self.req.request = Empty()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class MarkerArrayPublisher(Node):
    def __init__(self, debug : bool):
        super().__init__('marker_array_publisher')
        self.DEBUG = debug
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

    def publish(self, msg : MarkerArray):
        self.publisher.publish(msg)

class MarkerPublisher(Node):
    def __init__(self, debug : bool):
        super().__init__('marker_publisher')
        self.DEBUG = debug
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

    def publish(self, msg : Marker):
        self.publisher.publish(msg)

class JointStatesSubscriber(Node):
    """
    Canonical subsciption didn't work for some reason - I was not gettgin any updates in the controller.
    # qos = QoSProfile(
    #     reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    #     durability=rclpy.qos.DurabilityPolicy.VOLATILE,
    #     history=rclpy.qos.HistoryPolicy.SYSTEM_DEFAULT,
    #     depth=10
    # )

    # self.subscription = self.create_subscription(
    #     JointState,
    #     'joint_states',
    #     cb,
    #     qos)
    # self.subscription  # prevent unused variable warning
    """
    def __init__(self, debug : bool, callback, sleep : float = 0.1):
        super().__init__('joint_states_subscriber')
        self.DEBUG = debug
        self.SLEEP = sleep

        self.callback = callback
        self.subscriber = util.get_joint_states_subscriber()
        self.cancel = threading.Event()
        loop = asyncio.get_event_loop()
        self.subscriber_thread = threading.Thread(target=self._loop_in_thread, args=(loop,))
        self.subscriber_thread.start()

    def destroy(self):
        self.cancel.set()
        self.subscriber.terminate()
        self.subscriber_thread.join()

    def _loop_in_thread(self, loop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.read_joint_states())

    @asyncio.coroutine
    def read_joint_states(self):
        for js in util.get_joint_states(self.subscriber):
            self.callback(js)
            if self.cancel.is_set():
                return
            yield from asyncio.sleep(self.SLEEP)