#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "auna_msgs/msg/string_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
      std::chrono::milliseconds(100),
      std::bind(&MarkerPositionNode::publish_marker_positions, this));
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
    all_marker_positions.header.frame_id = "map";

    for (const auto & robot_namespace : robot_names_) {
      for (const std::string & marker_name :
           {"marker_1", "marker_2", "marker_3", "marker_4", "marker_5", "marker_6", "marker_7"}) {
        try {
          // Get transform from gazebo_world to ground_truth_base_link
          geometry_msgs::msg::TransformStamped tf_world_to_gt = tf_buffer_->lookupTransform(
            "gazebo_world", robot_namespace + "/ground_truth_base_link", tf2::TimePointZero);

          tf2::Transform tf_world_to_gt_transform;
          tf2::fromMsg(tf_world_to_gt.transform, tf_world_to_gt_transform);

          // Get transform from base_link to marker
          geometry_msgs::msg::TransformStamped tf_base_to_marker = tf_buffer_->lookupTransform(
            robot_namespace + "/base_link", robot_namespace + "/" + marker_name,
            tf2::TimePointZero);

          tf2::Transform tf_base_to_marker_transform;
          tf2::fromMsg(tf_base_to_marker.transform, tf_base_to_marker_transform);

          // Combine the transforms to get marker position in gazebo_world
          tf2::Transform tf_marker_in_world =
            tf_world_to_gt_transform * tf_base_to_marker_transform;

          geometry_msgs::msg::Pose marker_pose_in_world;
          marker_pose_in_world.position.x = tf_marker_in_world.getOrigin().x();
          marker_pose_in_world.position.y = tf_marker_in_world.getOrigin().y();
          marker_pose_in_world.position.z = tf_marker_in_world.getOrigin().z();
          marker_pose_in_world.orientation = tf2::toMsg(tf_marker_in_world.getRotation());

          RCLCPP_DEBUG(
            this->get_logger(), "marker in gazebo_world: x=%.2f, y=%.2f, z=%.2f",
            marker_pose_in_world.position.x, marker_pose_in_world.position.y,
            marker_pose_in_world.position.z);

          all_marker_positions.poses.push_back(marker_pose_in_world);
        } catch (const tf2::TransformException & ex) {
          // RCLCPP_WARN(
          //   this->get_logger(), "Could not get transform for %s/%s: %s", robot_namespace.c_str(),
          //   marker_name.c_str(), ex.what());
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
