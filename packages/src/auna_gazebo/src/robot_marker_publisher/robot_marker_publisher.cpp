#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "auna_msgs/msg/string_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class MarkerPositionNode : public rclcpp::Node
{
public:
  MarkerPositionNode() : Node("marker_position_node")
  {
    // TF setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to robot names
    robot_name_subscription_ = this->create_subscription<auna_msgs::msg::StringArray>(
      "/robot_names", 10,
      std::bind(&MarkerPositionNode::robot_name_callback, this, std::placeholders::_1));

    // Publisher for all marker positions
    all_markers_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("/all_marker_positions", 10);

    // Timer to regularly publish all marker positions
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MarkerPositionNode::publish_marker_positions, this));
  }

private:
  void robot_name_callback(const auna_msgs::msg::StringArray::SharedPtr msg)
  {
    robot_names_ = msg->strings;
  }

  void publish_marker_positions()
  {
    geometry_msgs::msg::PoseArray all_marker_positions;
    all_marker_positions.header.stamp = this->now();
    all_marker_positions.header.frame_id = "world";

    for (const auto & robot_namespace : robot_names_) {
      for (const std::string & marker_name :
           {"marker_1", "marker_2", "marker_3", "marker_4", "marker_5", "marker_6", "marker_7"}) {
        std::string target_frame = robot_namespace + "/odom";
        std::string source_frame = robot_namespace + "/" + marker_name;

        try {
          // Check if the transform exists
          if (!tf_buffer_->canTransform(target_frame, source_frame, rclcpp::Time(0))) {
            RCLCPP_WARN(
              this->get_logger(), "Transform not available for %s/%s", robot_namespace.c_str(),
              marker_name.c_str());
            continue;  // Skip this marker
          }
          geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0));

          geometry_msgs::msg::Pose pose;
          pose.position.x = transform.transform.translation.x;
          pose.position.y = transform.transform.translation.y;
          pose.position.z = transform.transform.translation.z;
          pose.orientation.w = 1.0;

          all_marker_positions.poses.push_back(pose);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN(
            this->get_logger(), "Could not get transform for %s/%s: %s", robot_namespace.c_str(),
            marker_name.c_str(), ex.what());
        }
      }
    }

    all_markers_pub_->publish(all_marker_positions);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<auna_msgs::msg::StringArray>::SharedPtr robot_name_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_markers_pub_;
  std::vector<std::string> robot_names_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPositionNode>());
  rclcpp::shutdown();
  return 0;
}
