from ur_controller import deus_ex_cubus as dec
import rclpy
import sys

def main(args=None):
    rclpy.init(args=args)

    simulation = True
    easter_egg = False

    if len(sys.argv) >= 2:
        for arg in sys.argv[1:]:
            if arg == "simulation=false":
                simulation = False
            elif arg == "east=eregg":
                easter_egg = True

    if easter_egg:
        from ur_controller import egg
        egg.hatch()

    config = dec.DeusExCubusConfig()
    config.debug=True
    config.simulation=simulation
    config.pack_commands=True
    config.blending_radius=0.1
    config.acceleration=1.0
    config.velocity=1.0
    config.box_spacing = 0.003 # meters
    config.pre_pick_z_offset = 2.0
    config.use_ascii_art = True
    config.placing_speed_factors = [1.0, 0.8, 0.6, 0.4, 0.3, 0.2, 0.1, 0.1]

    if config.use_ascii_art:
        from ur_controller import banner
        banner.print_banner()
        banner.print_panel("A final project for Ocado Robotics Accelerator 2022")

    config.print()

    print(f"Creating DeusExCubus targeting {'simulator' if simulation else 'UR10e'}.")

    deus = dec.DeusExCubus(config)
    deus.init()

    deus.build_stairway_to_heaven()
    deus.build_tower_of_babylon()

    # Shut down
    deus.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()