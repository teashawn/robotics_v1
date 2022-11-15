from ur_controller.service_clients import URScriptClientAsync, GetEefAngleAxisClientAsync, PowerOnClientAsync, BrakeReleaseClientAsync
from ur_controller import constants

import os

# DeusExCubus
# OcadoTeaParty
# CubeAddict
# BoxMaster, MasterBox
class BoxMind:
    def __init__(self, debug : bool = False, simulation : bool = True):
        self.DEBUG = debug
        self.SIMULATION = simulation

        self.command_client = URScriptClientAsync(debug=debug)
        self.eef_angle_axis_client = GetEefAngleAxisClientAsync(debug=debug)
        self.power_on_client = PowerOnClientAsync(debug=debug)
        self.brake_release_client = BrakeReleaseClientAsync(debug=debug)

    def init(self):
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
            "box_8",
            "box_11",
            "box_3"
        ]

        # move boxes
        for box in BOXES:
            print(f"Picking {box}.")

            # move above box
            self.command_client.send_robot_request(constants.PRE_PICK_WAYPOINTS[box].as_movel())

            # open gripper
            if not self.SIMULATION:
                self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

            # grab box
            self.command_client.send_robot_request(constants.WAYPOINTS[box].as_movel())

            # close gripper
            if not self.SIMULATION:
                self.command_client.send_gripper_request(constants.GRIPPER_CLOSE_COMMAND)

            print(f"Placing {box}.")

            # move to target position
            self.command_client.send_robot_request(constants.DESTINATIONS[box].as_movel())

            # open gripper
            if not self.SIMULATION:
                self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

            # move above box
            self.command_client.send_robot_request(constants.POST_PLACE_WAYPOINTS[box].as_movel())

        print("Stariway to heaven ready.")

    def ho(self):
        response = self.eef_angle_axis_client.send_request()
        print(response.success)
        print(response.angle_axis)
        print("Tropa")