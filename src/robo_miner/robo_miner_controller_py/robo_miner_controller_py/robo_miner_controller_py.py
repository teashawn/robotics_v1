from robo_miner_controller_py import service_clients, models, map_explorer
import rclpy

"""
Notes:
- count number of turns to detect looping
"""

def main(args=None):
    rclpy.init(args=args)

    # State
    explorer = map_explorer.MapExplorer(debug=True)

    # Traverse map
    explorer.reveal_map()

    # Navigate to specific coordinates
    #destination = models.MapNode(5,6)
    #explorer.navigate(destination)

    # Shut down
    rclpy.shutdown()


if __name__ == '__main__':
    main()