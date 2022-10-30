from robo_miner_controller_py import map_explorer as mp
import rclpy

def main(args=None):
    rclpy.init(args=args)

    # State
    explorer = mp.MapExplorer(
        debug=False,
        use_turn_aware_pathfinding=True
    )

    # Authenticate
    explorer.authenticate(
        "tisho@taxime.to",
        "https://github.com/teashawn/robotics_v1",
        "6557d62cd96857dcff990b121370bb5a0636b88c"
    )

    # Traverse map
    explorer.reveal_map()

    # Validate map
    explorer.validate_map()

    # Validate longest sequence
    explorer.validate_longest_sequence()

    # Mine longest sequence
    explorer.mine_longest_sequence()

    # Shut down
    rclpy.shutdown()


if __name__ == '__main__':
    main()