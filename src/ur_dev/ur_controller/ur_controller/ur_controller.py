from ur_controller import box_mind as bm
import rclpy
from autologging import TRACE
import logging, sys
logging.basicConfig(level=TRACE, stream=sys.stdout, format="\n%(funcName)s:%(message)s\n") # %(levelname)s:%(name)s:%(funcName)s:%(message)s

def main(args=None):
    rclpy.init(args=args)

    simulation = True
    if len(sys.argv) >= 2:
        # We target the real UR10e only if the simulation flag is explicitly lowered
        simulation = not (sys.argv[1] == "simulation=false")

    print(f"Creating BoxMind targeting {'simulator' if simulation else 'UR10e'}.")

    mind = bm.BoxMind(
        debug=True,
        simulation=simulation
    )

    mind.yo()

    # Shut down
    rclpy.shutdown()

if __name__ == '__main__':
    main()