from ur_controller.service_clients import URScriptClientAsync, GetEefAngleAxisClientAsync
from ur_controller import constants

import time

class BoxMind:
    def __init__(self, debug : bool = False, simulation : bool = True):
        self.DEBUG = debug
        self.SIMULATION = simulation

        self.command_client = URScriptClientAsync(debug=debug)
        self.eef_angle_axis_client = GetEefAngleAxisClientAsync(debug=debug)

    def yo(self):
        p = constants.WAYPOINTS["home"]
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

        time.sleep(5)

        self.command_client.send_request(p.as_movel())
        if self.SIMULATION:
            self.command_client.send_request(constants.GRIPPER_ACTIVATE_COMMAND)

        for box in BOXES:
            print(f"Picking {box}")
            self.command_client.send_request(constants.PRE_PICK_WAYPOINTS[box].as_movel())

            if self.SIMULATION:
                self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

            self.command_client.send_request(constants.WAYPOINTS[box].as_movel())

            if self.SIMULATION:
                self.command_client.send_gripper_request(constants.GRIPPER_CLOSE_COMMAND)

            print(f"Placing {box}")
            self.command_client.send_request(constants.DESTINATIONS[box].as_movel())

            if self.SIMULATION:
                self.command_client.send_gripper_request(constants.GRIPPER_OPEN_COMMAND)

            self.command_client.send_request(constants.POST_PLACE_WAYPOINTS[box].as_movel())

        print("Opa")

    def ho(self):
        response = self.eef_angle_axis_client.send_request()
        print(response.success)
        print(response.angle_axis)
        print("Tropa")