from ur_controller.service_clients import URScriptClientAsync, GetEefAngleAxisClientAsync, PowerOnClientAsync, BrakeReleaseClientAsync, MarkerPublisher, MarkerArrayPublisher, JointStatesSubscriber
from ur_controller import constants, models
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg._time import Time
from builtin_interfaces.msg._duration import Duration
import rclpy

import os

class DeusExCubus:
    def __init__(self, debug : bool = False, simulation : bool = True, pack_commands : bool = False, blending_radius : float = 0.0, acceleration : float = 0.0, velocity : float = 0.0):
        self.DEBUG = debug
        self.SIMULATION = simulation
        self.PACK_COMMANDS = pack_commands
        self.BLENDING_RADIUS = blending_radius
        self.ACCELERATION = acceleration
        self.VELOCITY = velocity

        self.command_client = URScriptClientAsync(debug=debug)
        self.eef_angle_axis_client = GetEefAngleAxisClientAsync(debug=debug)
        self.power_on_client = PowerOnClientAsync(debug=debug)
        self.brake_release_client = BrakeReleaseClientAsync(debug=debug)
        self.marker_array_publisher = MarkerArrayPublisher(debug=debug)
        self.marker_publisher = MarkerPublisher(debug=debug)
        self.joint_states_subscriber = JointStatesSubscriber(debug=debug, cb=self._on_joint_states_changed)

    def init(self):
        if self.SIMULATION:
            print("Creating scene.")
            self._create_scene()

        print("Powering on.")
        response = self.power_on_client.send_request()
        if not response.success:
            print(f"Could not power on robot: {response.error_reason}")
            os.exit(1)

        print("Releasing brakes.")
        response = self.brake_release_client.send_request()
        if not response.success:
            print(f"Could not release brakes: {response.error_reason}")
            os.exit(1)

        print("Going to home position.")
        home = constants.WAYPOINTS["home"]
        self.command_client.send_robot_request(home.as_movel())
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_ACTIVATE_COMMAND)

    def _on_joint_states_changed(self, msg):
        # name, position
        print(f"Received joint states: {msg}")

    def _get_box_marker(self, b : models.Waypoint, id : int) -> Marker:
        # http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.command_client.get_clock().now().to_msg() #Time()
        marker.ns = "boxes"
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = constants.BOX_SIDE
        marker.scale.y = constants.BOX_SIDE
        marker.scale.z = constants.BOX_SIDE
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z= 0.0
        marker.pose.orientation.w = 1.0

        marker.pose.position.x = b.X
        marker.pose.position.y = b.Y
        marker.pose.position.z = b.Z
        
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    
        #marker.lifetime = Duration(sec=1000000000)

        return marker

    def _get_scene_markers(self):
        markers = []

        for b in [constants.WAYPOINTS[k] for k in constants.WAYPOINTS.keys() if "box" in k]:
            markers.append(self._get_box_marker(b, len(markers)))

        return markers

    def _create_scene(self):
        self.marker_array_publisher.publish(MarkerArray(markers=self._get_scene_markers()))

    def _move_box_packed(self, box):
        commands = []

        print(f"Picking {box}.")

        # move above box
        commands.append(constants.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # open gripper
        if not self.SIMULATION:
            commands.append(constants.GRIPPER_OPEN_COMMAND)

        # grab box
        commands.append(constants.WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # close gripper
        if not self.SIMULATION:
            commands.append(constants.GRIPPER_CLOSE_COMMAND)

        # move above box
        commands.append(constants.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        print(f"Placing {box}.")

        # move above box
        commands.append(constants.POST_PLACE_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # move to target position
        commands.append(constants.DESTINATIONS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # open gripper
        if not self.SIMULATION:
            commands.append(constants.GRIPPER_OPEN_COMMAND)

        # move above box
        commands.append(constants.POST_PLACE_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        packed_command = "\n".join(commands)

        if not self.SIMULATION:
            self.command_client.send_gripper_request(packed_command)
        else:
            self.command_client.send_robot_request(packed_command)

    def _move_box_unpacked(self, box):
        print(f"Picking {box}.")

        # move above box
        self.command_client.send_robot_request(constants.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # open gripper
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

        # grab box
        self.command_client.send_robot_request(constants.WAYPOINTS[box].as_movel())

        # close gripper
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_CLOSE_COMMAND)

        # move above box
        self.command_client.send_robot_request(constants.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        print(f"Placing {box}.")

        # move above box
        self.command_client.send_robot_request(constants.POST_PLACE_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # move to target position
        self.command_client.send_robot_request(constants.DESTINATIONS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # open gripper
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

        # move above box
        self.command_client.send_robot_request(constants.POST_PLACE_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

    def _move_box(self, box):
        if self.PACK_COMMANDS:
            self._move_box_packed(box)
        else:
            self._move_box_unpacked(box)

    def build_stairway_to_heaven(self):
        
        # movej([-1.570796327,-1.570796327,-1.570796327,-1.570796327,1.570796327,0],a=5.0,v=1.0)"

        BOXES = [
            "box_10",
            "box_1",
            "box_4",
            "box_13",
            "box_7",
            "box_2",
            "box_12",
            "box_5",
            "box_8",
            "box_11"
        ]

        # move boxes
        for box in BOXES:
            self._move_box(box)

        print("Stairway to heaven ready.")

    def build_tower_of_babylon(self):
        
        # movej([-1.570796327,-1.570796327,-1.570796327,-1.570796327,1.570796327,0],a=5.0,v=1.0)"

        BOXES = [
            "box_3",
            "box_6",
            "box_14",
            "box_9"
        ]

        # move boxes
        for box in BOXES:
            self._move_box(box)

        print("Tower of Babylon ready.")

    def ho(self):
        response = self.eef_angle_axis_client.send_request()
        print(response.success)
        print(response.angle_axis)
        print("Tropa")