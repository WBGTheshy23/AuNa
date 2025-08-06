#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class PoseErrorNode : public rclcpp::Node
{
public:
  PoseErrorNode() : Node("pose_error_node")
  {
    // // Open CSV file for writing
    // csv_file_.open("pose_errors.csv", std::ios::out | std::ios::trunc);
    // if (csv_file_.is_open()) {
    //   csv_file_ << "timestamp,robot_name,position_error,yaw_error_deg\n";
    // } else {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing");
    // }

    for (const auto & robot_name : {"robot1", "robot2", "robot3"}) {
      std::string ekf_topic = "/" + std::string(robot_name) + "/global_pose";
      std::string gt_topic = "/" + std::string(robot_name) + "/gazebo_pose";

      // EKF pose subscriber
      auto ekf_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        ekf_topic, 10, [this, robot_name](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          ekf_poses_[robot_name] = *msg;
          computeError(robot_name);
        });

      // Ground truth pose subscriber
      auto gt_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        gt_topic, 10, [this, robot_name](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          gt_poses_[robot_name] = *msg;
          computeError(robot_name);
        });

      // Store subscribers to keep them alive
      ekf_subs_.push_back(ekf_sub);
      gt_subs_.push_back(gt_sub);
    }
  }

  // ~PoseErrorNode()
  // {
  //   if (csv_file_.is_open()) {
  //     csv_file_.close();
  //   }
  // }

private:
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> ekf_poses_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> gt_poses_;

  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> ekf_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> gt_subs_;

  // std::ofstream csv_file_;

  void computeError(const std::string & robot_name)
  {
    if (ekf_poses_.count(robot_name) == 0 || gt_poses_.count(robot_name) == 0) {
      return;  // wait for both poses to be available
    }

    const auto & ekf_pose = ekf_poses_[robot_name].pose;
    const auto & gt_pose = gt_poses_[robot_name].pose;

    // position error
    double dx = ekf_pose.position.x - gt_pose.position.x;
    double dy = ekf_pose.position.y - gt_pose.position.y;
    double position_error = std::sqrt(dx * dx + dy * dy);

    // Yaw error
    tf2::Quaternion q_ekf, q_gt;
    tf2::fromMsg(ekf_pose.orientation, q_ekf);
    tf2::fromMsg(gt_pose.orientation, q_gt);

    double roll1, pitch1, yaw1;
    double roll2, pitch2, yaw2;
    tf2::Matrix3x3(q_ekf).getRPY(roll1, pitch1, yaw1);
    tf2::Matrix3x3(q_gt).getRPY(roll2, pitch2, yaw2);

    double yaw_error = std::fabs(yaw1 - yaw2);
    yaw_error = std::fmod(yaw_error + M_PI, 2 * M_PI) - M_PI;  // wrap [-π, π]
    yaw_error = std::abs(yaw_error * 180.0 / M_PI);            // yaw error in degrees

    RCLCPP_INFO(
      this->get_logger(), "[%s] Pos error: %.3f m, Yaw error: %.2f°", robot_name.c_str(),
      position_error, yaw_error);

    // // Write to CSV file
    // if (csv_file_.is_open()) {
    //   rclcpp::Time now = this->get_clock()->now();
    //   csv_file_ << now.seconds() << "," << robot_name << "," << position_error << "," <<
    //   yaw_error
    //             << "\n";
    //   csv_file_.flush();
    // }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseErrorNode>());
  rclcpp::shutdown();
  return 0;
}
