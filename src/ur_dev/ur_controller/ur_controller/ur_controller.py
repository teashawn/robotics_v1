from ur_controller import deus_ex_cubus as dec
import rclpy
import sys

"""
TODO

+ generate constants file, based on confgurations from controller
+ patch unreachable box coordinates for simulation
+ extract box spacing as a parameter
+ consider running terminal topic echo and piping results to controller
  as a workaround for subscribers not getting data
+ make ascii art configurable

- use movej for placing boxes 14 and 9
- as plan B if gripper rotation doesn't work out, prepare alternative waypoints for
  building stairway perpendicular to the current one

- tower manouvre:
    - TCP orientation stays with value for Table B
    - wrist 2 from 90 to 0 degrees

- tower manouvre plan b:
    - pre-place waypoint in +X direction instead of +Z anf then place with a sideways motion,
    maybe with a little bigger box spacing

- lock joint states dict when reading or single read states?
- ensure last batched movel command has blend radius = 0
- check how box spacing is used
"""

def main(args=None):
    rclpy.init(args=args)

    simulation = True
    if len(sys.argv) >= 2:
        # We target the real UR10e only if the simulation flag is explicitly lowered
        simulation = not (sys.argv[1] == "simulation=false")

    config = dec.DeusExCubusConfig()
    config.debug=True
    config.simulation=simulation
    config.pack_commands=True
    config.blending_radius=0.1
    config.acceleration=1.0 #1.5?
    config.velocity=1.0
    config.box_spacing = 0.0025 # meters
    config.pre_pick_z_offset = 2.0
    config.use_ascii_art = True

    if config.use_ascii_art:
        from ur_controller import banner
        
        banner.print_banner()
        print("\n\n")
        banner.print_panel("A final project for Ocado Robotics Accelerator 2022")
        print("\n\n")

    config.print()

    print(f"Creating DeusExCubus targeting {'simulator' if simulation else 'UR10e'}.")

    deus = dec.DeusExCubus(config)
    deus.init()

    #deus.build_stairway_to_heaven()
    deus.build_tower_of_babylon()

    # Shut down
    deus.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()