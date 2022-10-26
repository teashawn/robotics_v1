#ifndef ROBO_MINER_CONTROLLER_ROBOMINERCONTROLLER_H_
#define ROBO_MINER_CONTROLLER_ROBOMINERCONTROLLER_H_

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include "robo_miner_interfaces/srv/query_initial_robot_position.hpp"
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_interfaces/msg/field_point.hpp"
#include "utils/ErrorCode.h"

class RoboMinerController : public rclcpp::Node {
public:
    RoboMinerController();
    ~RoboMinerController();

    ErrorCode getInitialPosition();
private:
    using Empty = std_msgs::msg::Empty;
    using String = std_msgs::msg::String;
    using RobotMove = robo_miner_interfaces::srv::RobotMove;
    using FieldPoint = robo_miner_interfaces::msg::FieldPoint;
    using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;

    ErrorCode initCommunication();

    rclcpp::Client<QueryInitialRobotPosition>::SharedPtr _initialRobotPosClient;
    rclcpp::Client<RobotMove>::SharedPtr _robotMoveClient;
};

#endif /* ROBO_MINER_CONTROLLER_ROBOMINERCONTROLLER_H_ */