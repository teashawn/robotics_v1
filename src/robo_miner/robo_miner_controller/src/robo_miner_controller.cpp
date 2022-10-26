// Corresponding header
#include "robo_miner_controller/RoboMinerController.h"

#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/empty.hpp>
#include "std_msgs/msg/string.hpp"

#include "utils/Log.h"

#include "robo_miner_interfaces/srv/query_initial_robot_position.hpp"
#include "robo_miner_interfaces/srv/robot_move.hpp"
#include "robo_miner_common/defines/RoboMinerTopics.h"

using namespace std::literals;
using namespace std::chrono_literals;

using Empty = std_msgs::msg::Empty;
using String = std_msgs::msg::String;
using RobotMove = robo_miner_interfaces::srv::RobotMove;
using FieldPoint = robo_miner_interfaces::msg::FieldPoint;
using QueryInitialRobotPosition = robo_miner_interfaces::srv::QueryInitialRobotPosition;

RoboMinerController::RoboMinerController() : Node("robo_miner_controller") {
  this->initCommunication();
}

RoboMinerController::~RoboMinerController() {

}

ErrorCode RoboMinerController::initCommunication() {
  using namespace std::placeholders;
  constexpr size_t queueSize = 10;
  const rclcpp::QoS qos(queueSize);

  _initialRobotPosClient = this->create_client<QueryInitialRobotPosition>(
    QUERY_INITIAL_ROBOT_POSITION_SERVICE
  );

  _robotMoveClient = this->create_client<RobotMove>(
    ROBOT_MOVE_SERVICE
  );

  return ErrorCode::SUCCESS;
}

ErrorCode RoboMinerController::getInitialPosition() {
  auto request = std::make_shared<QueryInitialRobotPosition::Request>();

  while (!this->_initialRobotPosClient->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return ErrorCode::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = this->_initialRobotPosClient->async_send_request(request);
  
  // result.get();
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Direction: %s", result.get()->robot_position_response.robot_dir);
  return ErrorCode::SUCCESS;

  // // Wait for the result.
  // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
  //   rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Direction: %s", result.get()->robot_position_response.robot_dir);
  //   return ErrorCode::SUCCESS;
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service initialRobotPos");
  //   return ErrorCode::FAILURE;
  // }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  auto controller = std::make_shared<RoboMinerController>();
  //controller->getInitialPosition();

  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Testicle");

  //printf("hello world robo_miner_controller package\n");
  

  
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
