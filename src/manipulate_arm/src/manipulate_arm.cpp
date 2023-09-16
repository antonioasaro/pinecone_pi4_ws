#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

class ManipulatorArmPublisher : public rclcpp::Node
{
  public:
    ManipulatorArmPublisher(): Node("manipulator_arm_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/robot_arm_controller/joint_trajectory", 10);
      timer_ = this->create_wall_timer(20000ms, std::bind(&ManipulatorArmPublisher::ManipulatorArmPublish, this));
    }

  private:
    void ManipulatorArmPublish()
    {
      trajectory_msgs::msg::JointTrajectory traj;

      traj.header.frame_id = "base_link";
      traj.joint_names.resize(6);
      traj.joint_names[0] ="base_waist_joint";
      traj.joint_names[1] ="shoulder_joint";
      traj.joint_names[2] ="elbow_joint";
      traj.joint_names[3] ="wrist_joint";
      traj.joint_names[4] ="gripper1_joint";
      traj.joint_names[5] ="gripper2_joint";

      traj.points.resize(2);

      traj.points[0].positions.resize(6);
      traj.points[0].positions[0] =  0.0;
      traj.points[0].positions[1] = -0.8;
      traj.points[0].positions[2] =  0.3;
      traj.points[0].positions[3] =  0.7;
      traj.points[0].positions[4] =  0.095;
      traj.points[0].positions[5] =  0.095;
      traj.points[0].time_from_start = rclcpp::Duration::from_seconds(10.0);

      traj.points[1].positions.resize(6);
      traj.points[1].positions[0] =  0.0;
      traj.points[1].positions[1] =  0.0;
      traj.points[1].positions[2] =  0.0;
      traj.points[1].positions[3] =  0.0;
      traj.points[1].positions[4] =  0.6;
      traj.points[1].positions[5] =  0.6;
      traj.points[1].time_from_start = rclcpp::Duration::from_seconds(20.0);
      
      RCLCPP_INFO(this->get_logger(), "Publishing joint_trajectory: %d", (int) count_++);
      publisher_->publish(traj);
    }
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

// int setValeurPoint(trajectory_msgs::JointTrajectory* trajectoire,int pos_tab, int val);

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManipulatorArmPublisher>());
    rclcpp::shutdown();
    return 0;

    // int i(100);

    // while(ros::ok()) {

    //         traj.header.stamp = ros::Time::now();

    //         for(int j=0; j<4; j++) {
    //                 setValeurPoint(&traj,j,i);
    //         }

    //         traj.points[0].time_from_start = ros::Duration(1);

    //         arm_pub.publish(traj);
    //         ros::spinOnce();

    //         loop_rate.sleep();
    //         i++;
    // }

    // return 0;
}

// int setValeurPoint(trajectory_msgs::JointTrajectory* trajectoire,int pos_tab, int val){
//     trajectoire->points[0].positions[pos_tab] = val;
//     return 0;
// }
