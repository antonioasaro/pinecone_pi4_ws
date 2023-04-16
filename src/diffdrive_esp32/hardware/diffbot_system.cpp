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

/*ROS2 Control Demos diffbot hardware interface publisher. My use-case is sending hardware commands from a Raspberry Pi 4 running a diffrential drive robot controller to a Teensy 4.1 microcontroller controlling the actual hardware. The Teensy is running micro-ROS and can subscribe and publish to topics over a USB connection to the Raspberry Pi. The important change between this gist and the official demos is the presence of a node class, and the addition of a shared pointer member for the node in DiffBotSystemHardware. The shared pointer gets assigned in the on_init() member function, at which point its publisher can be used.
 */

#include "ros2_control_demo_example_2/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>
using std::placeholders::_1;
//// #include <rclcpp_components/register_node_macro.hpp>

namespace ros2_control_demo_example_2
{

#ifdef ANTONIO
  Pi4_Esp32_Publisher::Pi4_Esp32_Publisher() : Node("pi4_esp32_publisher")
  {
    RCLCPP_INFO(this->get_logger(), "Starting 'Pi4_Esp32_Publisher'");
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("right_wheel_speed", 10);
  }

  void Pi4_Esp32_Publisher::Publish_Speed(const float_t speed) const
  {
    std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
    msg->data = (int32_t) (speed * 1000);
    // if (msg->data != 0) RCLCPP_INFO(this->get_logger(), "Publish speed of: %d", (int)msg->data);
    publisher_->publish(std::move(msg));
  }

  Pi4_Esp32_Subscriber::Pi4_Esp32_Subscriber() : Node("pi4_esp32_subscriber")
  {
    RCLCPP_INFO(this->get_logger(), "Starting 'Pi4_Esp32_Subscriber'");
    encoder_count_ = 0;
    subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "right_wheel_encoder", 10, std::bind(&Pi4_Esp32_Subscriber::Encoder_Callback, this, _1));
  }

  void Pi4_Esp32_Subscriber::Encoder_Callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
////    RCLCPP_INFO(this->get_logger(), "Subscribed encoder value: %d", msg->data);
    encoder_count_ = (int32_t)msg->data;
  }

  int32_t Pi4_Esp32_Subscriber::Encoder_Read()
  {
    return (encoder_count_);
  }
#endif

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    base_x_ = 0.0;
    base_y_ = 0.0;
    base_theta_ = 0.0;

#ifdef ANTONIO
    // Fire up the publisher node
    pi4_esp32_publisher_ = std::make_shared<Pi4_Esp32_Publisher>();
    // Fire up the subscriber node and keep it alive
    pi4_esp32_subscriber_ = std::make_shared<Pi4_Esp32_Subscriber>();
    executor_.add_node(pi4_esp32_subscriber_);
    std::thread([this]()
                { executor_.spin(); })
        .detach();
#endif

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

    for (auto i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    // set some default values
    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
      if (std::isnan(hw_positions_[i]))
      {
        hw_positions_[i] = 0;
        hw_velocities_[i] = 0;
        hw_commands_[i] = 0;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

    for (auto i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffBotSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    double radius = 0.02; // radius of the wheels
    double dist_w = 0.1;  // distance between the wheels
#ifdef ANTONIO
    double prev_pos;
    static double wheel_pos = 0;
    static double wheel_vel = 0;
#endif
    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      // Simulate DiffBot wheels's movement as a first-order system
      // Update the joint status: this is a revolute joint without any limit.
      // Simply integrates
#ifdef ANTONIO
#define COUNTS_PER_REV 1320

      auto new_time = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = new_time - time_;
      double deltasSeconds = diff.count();
      time_ = new_time;

      double rads_per_count = (2 * M_1_PI) / COUNTS_PER_REV;
      int32_t encoder_count = pi4_esp32_subscriber_->Encoder_Read();
      if (i == 1)
      {
        if (hw_velocities_[i] > 0)
        {
          // RCLCPP_INFO(
          //     rclcpp::get_logger("DiffBotSystemHardware"),
          //     "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
          //     hw_velocities_[i], info_.joints[i].name.c_str());

          prev_pos = wheel_pos;
          wheel_pos = encoder_count * rads_per_count;
          wheel_vel = (wheel_pos - prev_pos) / deltasSeconds;
          // RCLCPP_INFO(
          //     rclcpp::get_logger("DiffBotSystemHardware"),
          //     "wheel_pos %.5f and wheel_vel %.5f for '%s'!", wheel_pos,
          //     wheel_vel, info_.joints[i].name.c_str());
        }

        ////       hw_positions_[i] = wheel_pos;
        ////       hw_velocities_[i] = (wheel_pos - prev_pos) / deltasSeconds;
      }
      hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_commands_[i];
      hw_velocities_[i] = hw_commands_[i];
#else
      hw_positions_[i] = hw_positions_[1] + period.seconds() * hw_commands_[i];
      hw_velocities_[i] = hw_commands_[i];
#endif

#ifndef ANTONIO
      // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
      RCLCPP_INFO(
          rclcpp::get_logger("DiffBotSystemHardware"),
          "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
          hw_velocities_[i], info_.joints[i].name.c_str());
      // END: This part here is for exemplary purposes - Please do not copy to your production code
#endif
    }

    // Update the free-flyer, i.e. the base notation using the classical
    // wheel differentiable kinematics
    double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
    double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
    double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
    base_x_ += base_dx * period.seconds();
    base_y_ += base_dy * period.seconds();
    base_theta_ += base_dtheta * period.seconds();

#ifndef ANTONIO
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully read! (%.5f,%.5f,%.5f)",
        base_x_, base_y_, base_theta_);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
#endif

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ros2_control_demo_example_2 ::DiffBotSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
#ifndef ANTONIO
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");
#endif

    for (auto i = 0u; i < hw_commands_.size(); i++)
    {
#ifdef ANTONIO
      if (i == 0)
        pi4_esp32_publisher_->Publish_Speed(hw_commands_[i]); // publish to topic
#endif
#ifdef ANTONIO
      if (hw_commands_[i] != 0)
#endif    
      // Simulate sending commands to the hardware  
      RCLCPP_INFO(
          rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
          info_.joints[i].name.c_str());
    }
#ifndef ANTONIO
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code
#endif

    return hardware_interface::return_type::OK;
  }
} // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)
