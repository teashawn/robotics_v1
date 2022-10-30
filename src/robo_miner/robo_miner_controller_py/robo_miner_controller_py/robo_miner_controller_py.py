from robo_miner_controller_py import map_explorer as mp
import rclpy

def main(args=None):
    rclpy.init(args=args)

    # State
    explorer = mp.MapExplorer(
        debug=False,
        use_turn_aware_pathfinding=True
    )

    # Traverse map
    explorer.reveal_map()

    # Validate map
    explorer.validate_map()

    # Validate longest sequence
    explorer.validate_longest_sequence()

    # Mine map

    # Shut down
    rclpy.shutdown()


if __name__ == '__main__':
    main()