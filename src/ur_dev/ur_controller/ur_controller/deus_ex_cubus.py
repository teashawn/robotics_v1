from ur_controller.service_clients import URScriptClientAsync, GetEefAngleAxisClientAsync, PowerOnClientAsync, BrakeReleaseClientAsync, MarkerPublisher, MarkerArrayPublisher, JointStatesSubscriber
from ur_controller import constants, models, scene, util
from visualization_msgs.msg import MarkerArray, Marker

from threading import Lock
from math import degrees, radians
from copy import deepcopy
from typing import List

from rich import box
from rich.console import Console
from rich.table import Table

import os

class DeusExCubusConfig:
    def __init__(self):
        self.debug : bool = False
        self.simulation : bool = True
        self.pack_commands : bool = False
        self.blending_radius : float = 0.0
        self.acceleration : float = 0.0
        self.velocity : float = 0.0
        self.box_spacing : float = 0.0 # meters
        self.pre_pick_z_offset : float = 0.0 # boxes
        self.use_ascii_art : bool = True
        self.placing_speed_factors : List[float] = [] # speed factors per box Z-level

    def to_list(self):
        return [
            ["debug", self.debug],
            ["simulation", self.simulation],
            ["pack_commands", self.pack_commands],
            ["blending_radius", self.blending_radius],
            ["acceleration", self.acceleration],
            ["velocity", self.velocity],
            ["box_spacing", self.box_spacing],
            ["pre_pick_z_offset", self.pre_pick_z_offset],
            ["use_ascii_art", self.use_ascii_art],
            ["placing_speed_factors", self.placing_speed_factors]
        ]

    def print(self):
        table = Table(title="DeusExCubus Config", width=64)
        table.box = box.SQUARE

        table.add_column("Parameter")
        table.add_column("Value")

        table.columns[0].style = "bold bright_blue"
        table.columns[1].style = "bold green"

        for t in self.to_list():
            table.add_row(t[0], str(t[1]))

        console = Console()
        console.print(table)

class DeusExCubus:
    def __init__(self, config : DeusExCubusConfig):

        # Parameters
        self.DEBUG = config.debug
        self.SIMULATION = config.simulation
        self.PACK_COMMANDS = config.pack_commands
        self.BLENDING_RADIUS = config.blending_radius
        self.ACCELERATION = config.acceleration
        self.VELOCITY = config.velocity
        self.BOX_SPACING = config.box_spacing
        self.PRE_PICK_Z_OFFSET = config.pre_pick_z_offset
        self.PLACING_SPEED_FACTORS = config.placing_speed_factors
        
        # State
        self.JOINT_STATES = {}
        self.JOINT_STATES_LOCK = Lock()
        self.WAYPOINTS = scene.get_waypoints(simulation=self.SIMULATION)
        self.PRE_PICK_WAYPOINTS = scene.get_pre_pick_waypoints(self.WAYPOINTS, self.PRE_PICK_Z_OFFSET)
        self.DESTINATIONS = scene.get_destinations(self.WAYPOINTS, config.box_spacing)
        self.PRE_PLACE_WAYPOINTS = scene.get_pre_place_waypoints(self.DESTINATIONS, self.PRE_PICK_Z_OFFSET)

        # Messaging clients
        self.command_client = URScriptClientAsync(debug=config.debug)
        self.eef_angle_axis_client = GetEefAngleAxisClientAsync(debug=config.debug)
        self.power_on_client = PowerOnClientAsync(debug=config.debug)
        self.brake_release_client = BrakeReleaseClientAsync(debug=config.debug)
        self.marker_array_publisher = MarkerArrayPublisher(debug=config.debug)
        self.marker_publisher = MarkerPublisher(debug=config.debug)

        # Subscriber is commented out because it is receiving stale data for some reason
        self.joint_states_subscriber = None
        # self.joint_states_subscriber = JointStatesSubscriber(
        #     debug=config.debug,
        #     callback=self._on_joint_states_changed,
        #     sleep=0.1
        # )

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
        self.command_client.send_robot_request(f"movej({constants.HOME_POSITION_JOIN_STATES},a={self.ACCELERATION},v={self.VELOCITY})")
        home = self.WAYPOINTS["home"]
        self.command_client.send_robot_request(home.as_movel())

        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_ACTIVATE_COMMAND)

    def destroy(self):
        if self.joint_states_subscriber:
            self.joint_states_subscriber.destroy()

    def _on_joint_states_changed(self, msg):
        self.JOINT_STATES_LOCK.acquire()

        for i, n in enumerate(msg["name"]):
            self.JOINT_STATES[n] = msg["position"][i]

        self.JOINT_STATES_LOCK.release()

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

        for b in [self.WAYPOINTS[k] for k in self.WAYPOINTS.keys() if "box" in k]:
            markers.append(self._get_box_marker(b, len(markers)))

        return markers

    def _create_scene(self):
        self.marker_array_publisher.publish(MarkerArray(markers=self._get_scene_markers()))

    def _move_box_packed(self, box, placing_speed_factor):
        commands = []

        print(f"Picking {box}.")

        # move above box
        commands.append(self.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # open gripper
        if not self.SIMULATION:
            commands.append(constants.GRIPPER_OPEN_COMMAND)

        # grab box
        commands.append(self.WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=0))

        # close gripper
        if not self.SIMULATION:
            commands.append(constants.GRIPPER_CLOSE_COMMAND)

        # move above box
        commands.append(self.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        print(f"Placing {box}.")

        # move above box
        commands.append(self.PRE_PLACE_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # move to target position
        commands.append(self.DESTINATIONS[box].as_movel(a=self.ACCELERATION*placing_speed_factor, v=self.VELOCITY*placing_speed_factor, r=0))

        # open gripper
        if not self.SIMULATION:
            commands.append(constants.GRIPPER_OPEN_COMMAND)

        # move above box
        commands.append(self.PRE_PLACE_WAYPOINTS[box].as_movel(a=self.ACCELERATION*placing_speed_factor, v=self.VELOCITY*placing_speed_factor, r=self.BLENDING_RADIUS))

        packed_command = "\n".join(commands)

        if not self.SIMULATION:
            self.command_client.send_gripper_request(packed_command)
        else:
            self.command_client.send_robot_request(packed_command)

    def _move_box_unpacked(self, box, placing_speed_factor):
        print(f"Picking {box}.")

        # move above box
        self.command_client.send_robot_request(self.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # open gripper
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

        # grab box
        self.command_client.send_robot_request(self.WAYPOINTS[box].as_movel())

        # close gripper
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_CLOSE_COMMAND)

        # move above box
        self.command_client.send_robot_request(self.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        print(f"Placing {box}.")

        # move above box
        self.command_client.send_robot_request(self.PRE_PLACE_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        # move to target position
        self.command_client.send_robot_request(self.DESTINATIONS[box].as_movel(a=self.ACCELERATION*placing_speed_factor, v=self.VELOCITY*placing_speed_factor, r=0))

        # open gripper
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

        # move above box
        self.command_client.send_robot_request(self.PRE_PLACE_WAYPOINTS[box].as_movel(a=self.ACCELERATION*placing_speed_factor, v=self.VELOCITY*placing_speed_factor, r=self.BLENDING_RADIUS))

    def _move_box(self, box, placing_speed_factor):
        if self.PACK_COMMANDS:
            self._move_box_packed(box, placing_speed_factor)
        else:
            self._move_box_unpacked(box, placing_speed_factor)

    def _get_joint_states(self):
        p = util.get_joint_states_subscriber()
        states = util.get_joint_states_single(p)

        result = {}

        for i, n in enumerate(states["name"]):
            result[n] = states["position"][i]

        return result

    def _move_box_sideways(self, box):
        print(f"Picking {box}.")

        #################################################################
        # Step 1
        #################################################################
        print("-----> Step 1 [move above box]")
        #input()
        # move above box
        self.command_client.send_robot_request(self.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        #################################################################
        # Step 2
        #################################################################
        print("-----> Step 2 [open gripper]")
        #input()
        # open gripper
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

        #################################################################
        # Step 3
        #################################################################
        print("-----> Step 3 [grab box]")
        #input()
        # grab box
        self.command_client.send_robot_request(self.WAYPOINTS[box].as_movel())

        #################################################################
        # Step 4
        #################################################################
        print("-----> Step 4 [close gripper]")
        input()
        # close gripper
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_CLOSE_COMMAND)

        #################################################################
        # Step 5
        #################################################################
        print("-----> Step 5 [move above box]")
        input()
        # move above box
        self.command_client.send_robot_request(self.PRE_PICK_WAYPOINTS[box].as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=self.BLENDING_RADIUS))

        print(f"Placing {box}.")

        #################################################################
        # Step 6
        #################################################################
        print("-----> Step 6 [move sideways (X-axis) next to destination]")
        input()
        # move sideways (X-axis) next to final box position
        waypoint = deepcopy(self.DESTINATIONS[box])
        waypoint.X += constants.BOX_SIDE * 2
        waypoint.Z += self.BOX_SPACING * 2
        waypoint.RX, waypoint.RY, waypoint.RZ = deepcopy(constants.TABLE_B_ORIENTATION)

        print(f"X: {waypoint.X}")
        print(f"Y: {waypoint.Y}")
        print(f"Z: {waypoint.Z}")

        # no blending
        self.command_client.send_robot_request(waypoint.as_movel(a=self.ACCELERATION, v=self.VELOCITY, r=0))

        #################################################################
        # Step 7
        #################################################################
        print("-----> Step 7 [rotate wrist_2 -90 degrees]")
        input()
        # get current joint states
        # self.JOINT_STATES_LOCK.acquire()
        # joint_states = deepcopy(self.JOINT_STATES)
        # self.JOINT_STATES_LOCK.release()
        joint_states = self._get_joint_states()

        # rotate wrist_2 -90 degrees
        wrist_2_original = deepcopy(joint_states["wrist_2_joint"])
        joint_states["wrist_2_joint"] = radians(degrees(wrist_2_original) - 90)
        joint_values = [
            joint_states["shoulder_pan_joint"],
            joint_states["shoulder_lift_joint"],
            joint_states["elbow_joint"],
            joint_states["wrist_1_joint"],
            joint_states["wrist_2_joint"],
            joint_states["wrist_3_joint"]
        ]

        # move at half speed
        self.command_client.send_robot_request(f"movej({joint_values},a={self.ACCELERATION/2.0},v={self.VELOCITY/2.0})")

        #################################################################
        # Step 8
        #################################################################
        print("-----> Step 8 [move a little above destination]")
        input()
        # get changed TCP angle-axis rotation
        eef_angle_axis = self.eef_angle_axis_client.send_request().angle_axis

        # move a little above target position
        waypoint = deepcopy(self.DESTINATIONS[box])
        waypoint.Z += self.BOX_SPACING * 2
        waypoint.RX = eef_angle_axis.x
        waypoint.RY = eef_angle_axis.y
        waypoint.RZ = eef_angle_axis.z

        # move at half speed and no blending
        self.command_client.send_robot_request(waypoint.as_movel(a=self.ACCELERATION/2.0, v=self.VELOCITY/2.0, r=0))

        #################################################################
        # Step 9
        #################################################################
        print("-----> Step 9 [move to destination]")
        input()
        # move to target position
        waypoint = deepcopy(self.DESTINATIONS[box])
        waypoint.RX = eef_angle_axis.x
        waypoint.RY = eef_angle_axis.y
        waypoint.RZ = eef_angle_axis.z
        # move at quarter speed and no blending
        self.command_client.send_robot_request(waypoint.as_movel(a=self.ACCELERATION/4.0, v=self.VELOCITY/4.0, r=0))

        #################################################################
        # Step 10
        #################################################################
        print("-----> Step 10 [open gripper]")
        input()
        # open gripper
        if not self.SIMULATION:
            self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

        #################################################################
        # Step 11
        #################################################################
        print("-----> Step 11 [move a little above destination]")
        input()
        # get changed TCP angle-axis rotation
        eef_angle_axis = self.eef_angle_axis_client.send_request().angle_axis

        # move a little above target position
        waypoint = deepcopy(self.DESTINATIONS[box])
        waypoint.Z += self.BOX_SPACING * 2
        waypoint.RX = eef_angle_axis.x
        waypoint.RY = eef_angle_axis.y
        waypoint.RZ = eef_angle_axis.z
        # move at quarter speed and no blending
        self.command_client.send_robot_request(waypoint.as_movel(a=self.ACCELERATION/4.0, v=self.VELOCITY/4.0, r=0))

        #################################################################
        # Step 12
        #################################################################
        print("-----> Step 12 [move sideways (X-axis) next to destination]")
        input()
        # move sideways (X-axis) next to final box position
        waypoint = deepcopy(self.DESTINATIONS[box])
        waypoint.X += constants.BOX_SIDE * 2
        waypoint.Z += self.BOX_SPACING * 2
        waypoint.RX = eef_angle_axis.x
        waypoint.RY = eef_angle_axis.y
        waypoint.RZ = eef_angle_axis.z
        
        # move at half speed and no blending
        self.command_client.send_robot_request(waypoint.as_movel(a=self.ACCELERATION/2.0, v=self.VELOCITY/2.0, r=0))

        #################################################################
        # Step 13
        #################################################################
        print("-----> Step 13 [rotate wrist_2 to +90 degrees]")
        input()
        # get current joint states
        # self.JOINT_STATES_LOCK.acquire()
        # joint_states = deepcopy(self.JOINT_STATES)
        # self.JOINT_STATES_LOCK.release()
        joint_states = self._get_joint_states()

        # rotate wrist_2 +90 degrees
        wrist_2_original = deepcopy(joint_states["wrist_2_joint"])
        joint_states["wrist_2_joint"] = radians(degrees(wrist_2_original) + 90)
        joint_values = [
            joint_states["shoulder_pan_joint"],
            joint_states["shoulder_lift_joint"],
            joint_states["elbow_joint"],
            joint_states["wrist_1_joint"],
            joint_states["wrist_2_joint"],
            joint_states["wrist_3_joint"]
        ]
        # move at half speed
        self.command_client.send_robot_request(f"movej({joint_values},a={self.ACCELERATION/2.0},v={self.VELOCITY/2.0})")

    def build_stairway_to_heaven(self):
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

        BOX_Z_LEVELS = {
            "box_10": 1,
            "box_1": 1,
            "box_4": 2,
            "box_13": 1,
            "box_7": 2,
            "box_2": 3,
            "box_12": 1,
            "box_5": 2,
            "box_8": 3,
            "box_11": 4
        }

        # move boxes
        for box in BOXES:
            self._move_box(box, self.PLACING_SPEED_FACTORS[BOX_Z_LEVELS[box]-1])

        print("Stairway to heaven ready.")

    def build_tower_of_babylon(self):

        # safely in reach with approach from above
        BOXES = [
            "box_3",
            "box_6"
        ]

        BOX_Z_LEVELS = {
            "box_3": 5,
            "box_6": 6,
        }

        # move boxes
        for box in BOXES:
            self._move_box(box, self.PLACING_SPEED_FACTORS[BOX_Z_LEVELS[box]-1])

        # TODO: figure out `movej` problem with last 2 boxes
        # # unreachable from above
        # BOXES = [
        #     "box_14",
        #     "box_9"
        # ]

        # # move boxes
        # for box in BOXES:
        #     self._move_box_sideways(box)

        print("Tower of Babylon ready.")