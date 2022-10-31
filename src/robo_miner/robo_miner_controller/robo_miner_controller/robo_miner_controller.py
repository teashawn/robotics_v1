from robo_miner_controller import map_explorer as mp
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
        "ba30b5cafbf2e74fe84b136cfb56f5e5ababb6fc"
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