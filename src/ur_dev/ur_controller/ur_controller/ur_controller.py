from ur_controller import deus_ex_cubus as dec
import rclpy
from autologging import TRACE
import logging, sys
#logging.basicConfig(level=TRACE, stream=sys.stdout, format="\n%(funcName)s:%(message)s\n") # %(levelname)s:%(name)s:%(funcName)s:%(message)s

"""
TODO

- use movej for placing boxes 14 and 9
? generate constants file, based on confgurations from controller
- patch unreachable box coordinates for simulation
- extract box spacing as a parameter
- consider running terminal topic echo and piping results to controller
  as a workaround for subscribers not getting data

- as plan B if gripper rotation doesn't work out, prepare alternative waypoints for
  building stairway perpendicular to the current one

- tower manouvre:
    - TCP orientation stays with value for Table B
    - wrist 2 from 90 to 0 degrees

- tower manouvre plan b:
    - pre-place waypoint in +X direction instead of +Z anf then place with a sideways motion,
    maybe with a little bigger box spacing

- lock joint states dict when reading or single read states?
"""

def main(args=None):
    rclpy.init(args=args)

    simulation = True
    if len(sys.argv) >= 2:
        # We target the real UR10e only if the simulation flag is explicitly lowered
        simulation = not (sys.argv[1] == "simulation=false")

    print(f"Creating DeusExCubus targeting {'simulator' if simulation else 'UR10e'}.")

    deus = dec.DeusExCubus(
        debug=True,
        simulation=simulation,
        pack_commands=True,
        blending_radius=0.1,
        acceleration=1.0, #1.5?
        velocity=1.0
    )

    if simulation:
        deus.init()

    deus.build_stairway_to_heaven()
    #deus.build_tower_of_babylon()

    # Shut down
    deus.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()