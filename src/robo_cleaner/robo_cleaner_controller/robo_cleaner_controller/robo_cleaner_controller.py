from robo_cleaner_controller import map_explorer as mp
import rclpy
from autologging import TRACE
import logging, sys
logging.basicConfig(level=TRACE, stream=sys.stdout, format="\n%(funcName)s:%(message)s\n") # %(levelname)s:%(name)s:%(funcName)s:%(message)s

def main(args=None):
    rclpy.init(args=args)

    # State
    explorer = mp.MapExplorer(
        debug=True,
        use_turn_aware_pathfinding=True,
        retardation = 0 # sleeping between move commands seems to somewhat alleviate crashes in `robo_cleaner_gui`
    )

    # Authenticate
    explorer.authenticate(
        "tisho@taxime.to",
        "https://github.com/teashawn/robotics_v1",
        "b6c1a5e9fcc2c6c7226eab4892bbd280214818e6"
    )

    # # Traverse map
    explorer.reveal_map()

    # Shut down
    rclpy.shutdown()

if __name__ == '__main__':
    main()