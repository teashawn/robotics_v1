from robo_miner_controller_py import map_explorer as mp
import rclpy

def main(args=None):
    rclpy.init(args=args)

    # State
    explorer = mp.MapExplorer(
        debug=True,
        use_turn_aware_pathfinding=True
    )

    # Traverse map
    explorer.reveal_map()

    # Validate map

    # Mine map

    # Shut down
    rclpy.shutdown()


if __name__ == '__main__':
    main()