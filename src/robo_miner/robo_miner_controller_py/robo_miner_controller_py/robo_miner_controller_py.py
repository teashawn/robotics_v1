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

    # Service clients
    initial_position_client = service_clients.QueryInitialRobotPositionClientAsync(False)
    move_client = service_clients.RobotMoveClientAsync(False)
    validate_client = service_clients.FieldMapValidateClientAsync(False)
    
    # Get initial position
    response = initial_position_client.query()
    res = explorer.init(response)

    # Traverse map
    while res.next_step != models.MapMoveResult.FINISH:
        if res.next_step == models.MapMoveResult.BACKTRACK:
            # Turn around
            move_response = move_client.move(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT)
            res = explorer.update(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT, move_response)
            move_response = move_client.move(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT)
            res = explorer.update(models.ROBOT_MOVE_TYPE.ROTATE_RIGHT, move_response)
        else:
            move_response = move_client.move(res.move_type)
            res = explorer.update(res.move_type, move_response)

    print(f"Revealed map in {move_client.moves} moves.")

    # Navigate to specific coordinates
    destination = models.MapNode(5,6)
    steps = explorer.get_path(destination)
    print(f"Steps: {steps}")
    moves = explorer.get_moves(steps)
    print(f"Moves: {moves}")

    for m in moves:
        move_response = move_client.move(m)
        explorer.update(m, move_response)

    # Shut down
    initial_position_client.destroy_node()
    move_client.destroy_node()
    validate_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()