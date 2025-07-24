#include "op3_webots_ros2/op3_player_state.hpp"

using game_controller_msgs::msg::GameControlData;
using game_controller_msgs::msg::GameControlReturnData;

PlayerState::PlayerState(const rclcpp::Node::SharedPtr &node)
{
  team_number_ = node->get_parameter("team_number").as_int();
  player_number_ = node->get_parameter("player_number").as_int();

  RCLCPP_INFO(node->get_logger(), "PlayerState initialized with team=%d, player=%d",
              team_number_, player_number_);

  control_data_sub_ = node->create_subscription<GameControlData>(
    "/game_controller/control_data", 10,
    [this, node](const GameControlData::SharedPtr msg)
    {
      for (const auto &team : msg->teams)
      {
        if (team.team_number == team_number_)
        {
          GameControlReturnData return_msg;
          return_msg.team = team_number_;
          return_msg.player = player_number_;
          return_msg.message = 2; // ALIVE

          return_data_pub_->publish(return_msg);
          break;
        }
      }
    });

  return_data_pub_ = node->create_publisher<GameControlReturnData>(
    "/game_controller/return_data", 10);
}
