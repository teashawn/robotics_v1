from ur_controller import deus_ex_cubus as dec
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

    print(f"Creating DeusExCubus targeting {'simulator' if simulation else 'UR10e'}.")

    deus = dec.DeusExCubus(
        debug=True,
        simulation=simulation
    )

    if simulation:
        deus.init()

    deus.build_stairway_to_heaven()
    deus.build_tower_of_babylon()
    #mind.ho()

    # Shut down
    rclpy.shutdown()

if __name__ == '__main__':
    main()