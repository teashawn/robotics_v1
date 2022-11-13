#from rclpy.qos import qos_profile_default, qos_profile_sensor_data
#from rclpy.qos import QoSPresetProfiles
# SENSOR_DATA, SERVICES_DEFAULT, SYSTEM_DEFAULT
# https://docs.ros2.org/foxy/api/rclpy/api/qos.html

from urscript_interfaces.srv import UrScript
from std_msgs.msg import Empty
from rclpy.node import Node
import rclpy

from ur_controller import constants

def wrap_urscript(payload : str) -> str:
    return f"{constants.FUNC_HEADER}{'  '}{'  '.join([l.strip() for l in payload.splitlines()])}{constants.FUNC_FOOTER}"

class URScriptClientAsync(Node):

    def __init__(self, debug : bool):
        super().__init__('urscript_client_async')
        self.DEBUG = debug
        self.cli = self.create_client(
            UrScript,
            'urscript_service'
        )
        """
        ,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        """
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = UrScript.Request()

    def send_request(self, payload : str):
        self.req.data = wrap_urscript(payload)
        if self.DEBUG:
            print(f"Sending URScript: {self.req.data}")
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    # def query(self):
    #     response = self.send_request()

    #     if self.DEBUG:
    #         # Log
    #         success = response.success
    #         error_reason = response.error_reason
    #         state = response.initial_robot_state
    #         robot_dir = models.ROBOT_DIRECTION(state.robot_dir)
    #         initial_tile = state.robot_tile
    #         self.get_logger().info(
    #             f"\nSuccess: {success},\nError reason: {error_reason},\nDirection: {robot_dir},\nInitial tile: {initial_tile}\n"
    #         )
    #     return response