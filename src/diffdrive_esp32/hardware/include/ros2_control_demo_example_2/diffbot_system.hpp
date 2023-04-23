#define ANTONIO
// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_demo_example_2/visibility_control.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
using std::placeholders::_1;

namespace ros2_control_demo_example_2
{

#ifdef ANTONIO
  // The node definition for the publisher to talk to micro-ROS agent
  class Pi4_Esp32_Publisher : public rclcpp::Node
  {
  public:
    Pi4_Esp32_Publisher();
    void Publish_Speed(const float_t left_wheel_speed, const float_t right_wheel_speed) const;

  private:
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  };

  class Pi4_Esp32_Subscriber : public rclcpp::Node
  {
  public:
    Pi4_Esp32_Subscriber();
    int32_t Encoder_Read();

  private:
    int32_t encoder_count_;
    void Encoder_Callback(const std_msgs::msg::Int32::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
  };

#endif

  class DiffBotSystemHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

    ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    ROS2_CONTROL_DEMO_EXAMPLE_2_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

#ifdef ANTONIO
    // Make the publisher node a member
    std::shared_ptr<Pi4_Esp32_Publisher> pi4_esp32_publisher_;

    // Make the subscriber node a member
    std::shared_ptr<Pi4_Esp32_Subscriber> pi4_esp32_subscriber_;
#endif

  private:
    // Parameters for the DiffBot simulation
    double hw_start_sec_;
    double hw_stop_sec_;

    // Store the command for the simulated robot
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;

    // Store the wheeled robot position
    double base_x_, base_y_, base_theta_;

#ifdef ANTONIO
    std::chrono::time_point<std::chrono::system_clock> time_;
    rclcpp::executors::SingleThreadedExecutor executor_;
#endif
  };

} // namespace ros2_control_demo_example_2

#endif // ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
