from robo_miner_controller_py import service_clients, models, map_explorer
import rclpy

"""
Notes:
- count number of turns to detect looping
"""

def main(args=None):
    rclpy.init(args=args)

    # State
    explorer = map_explorer.MapExplorer(True)
    explorer.print_state()

    # Service clients
    query_intial_robot_position_client = service_clients.QueryInitialRobotPositionClientAsync()
    robot_move_client = service_clients.RobotMoveClientAsync()
    
    # Get initial position
    response = query_intial_robot_position_client.query()
    res = explorer.init(response)

    while res.next_step != models.MapMoveResult.FINISH:
        if res.next_step == models.MapMoveResult.BACKTRACK:
            # Turn around
            move_response = robot_move_client.move(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT)
            res = explorer.update(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT, move_response)
            move_response = robot_move_client.move(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT)
            res = explorer.update(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT, move_response)
        else:
            move_response = robot_move_client.move(res.move_type)
            res = explorer.update(res.move_type, move_response)

    print(f"Revealed map in {robot_move_client.moves} moves.")

    destination = models.MapNode(5,6)
    print(explorer.get_path(destination))

    # Shut down
    query_intial_robot_position_client.destroy_node()
    robot_move_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()