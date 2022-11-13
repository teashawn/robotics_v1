from ur_controller.service_clients import URScriptClientAsync

class BoxMind:
    def __init__(self, debug : bool = False, simulation : bool = True):
        self.DEBUG = debug
        self.SIMULATION = simulation

        self.command_client = URScriptClientAsync(debug=debug)

    def yo(self):
        response = self.command_client.send_request("movej([-1.570796327,-1.570796327,-1.570796327,-1.570796327,1.570796327,0],a=5.0,v=1.0)")
        print(response.success)
        print(response.error_reason)
        print("Opa")