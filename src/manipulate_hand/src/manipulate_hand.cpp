#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

using ActionGripperCommand = control_msgs::action::GripperCommand;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("manipulator_hand_client");

  // Create a client for the Hello action
  auto client = rclcpp_action::create_client<ActionGripperCommand>(
      node, "/robot_hand_controller/gripper_cmd");

  // Wait for the action server to become available
  if (!client->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  for (int i = 0; i < 2; i++)
  {
    RCLCPP_INFO(node->get_logger(), "Iteration %d", i);
    // Create a goal message
    auto goal = ActionGripperCommand::Goal();
    goal.command.position = i * 0.8;

    // Send the goal and wait for the result
    auto send_goal_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
      return 1;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
      return 1;
    }

    // Wait for the result
    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to get result");
      return 1;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(node->get_logger(), "Result: %d", result.result->reached_goal);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Action did not succeed");
    }

    sleep(10);
  }
  rclcpp::shutdown();
  return 0;
}