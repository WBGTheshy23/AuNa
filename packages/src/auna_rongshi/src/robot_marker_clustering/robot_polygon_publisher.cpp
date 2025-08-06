#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <unordered_map>
#include <vector>

class CarPolygonPublisherNode : public rclcpp::Node
{
public:
  CarPolygonPublisherNode() : Node("car_polygon_publisher_node")
  {
    // Subscribe to car orientations
    car_orientations_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/car_orientations", 10,
      std::bind(&CarPolygonPublisherNode::carOrientationsCallback, this, std::placeholders::_1));

    // Subscribe to car IDs
    car_id_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/car_id", 10,
      std::bind(&CarPolygonPublisherNode::carIdCallback, this, std::placeholders::_1));

    RCLCPP_DEBUG(this->get_logger(), "Car Polygon Publisher Node started.");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr car_orientations_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr car_id_subscription_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr>
    polygon_publishers_;

  void carOrientationsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    int car_id = 0;  // Unique ID for each car

    for (const auto & pose : msg->poses) {
      // Extract orientation (yaw) from the pose
      tf2::Quaternion quaternion;
      tf2::fromMsg(pose.orientation, quaternion);
      double roll, pitch, yaw;
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

      // Define the rectangle's four corners based on the car's center and orientation
      geometry_msgs::msg::Point32 corner1, corner2, corner3, corner4;

      // Rectangle dimensions (adjust as needed)
      double length = 0.4;  // Distance between front and rear wheels
      double width = 0.3;   // Distance between left and right wheels

      // Calculate the corners relative to the car's center
      double cos_yaw = std::cos(yaw);
      double sin_yaw = std::sin(yaw);

      corner1.x = pose.position.x + (-length / 2 * cos_yaw - width / 2 * sin_yaw);
      corner1.y = pose.position.y + (-length / 2 * sin_yaw + width / 2 * cos_yaw);
      corner1.z = pose.position.z;

      corner2.x = pose.position.x + (-length / 2 * cos_yaw + width / 2 * sin_yaw);
      corner2.y = pose.position.y + (-length / 2 * sin_yaw - width / 2 * cos_yaw);
      corner2.z = pose.position.z;

      corner3.x = pose.position.x + (length / 2 * cos_yaw + width / 2 * sin_yaw);
      corner3.y = pose.position.y + (length / 2 * sin_yaw - width / 2 * cos_yaw);
      corner3.z = pose.position.z;

      corner4.x = pose.position.x + (length / 2 * cos_yaw - width / 2 * sin_yaw);
      corner4.y = pose.position.y + (length / 2 * sin_yaw + width / 2 * cos_yaw);
      corner4.z = pose.position.z;

      // Create the polygon message
      geometry_msgs::msg::PolygonStamped polygon_msg;
      polygon_msg.header.frame_id = "gazebo_world";
      polygon_msg.header.stamp = this->get_clock()->now();

      polygon_msg.polygon.points.push_back(corner1);
      polygon_msg.polygon.points.push_back(corner2);
      polygon_msg.polygon.points.push_back(corner3);
      polygon_msg.polygon.points.push_back(corner4);
      polygon_msg.polygon.points.push_back(corner1);  // Close the polygon

      // Create a unique topic for each car
      std::string topic_name = "/car_" + std::to_string(car_id) + "_polygon";

      // Check if publisher already exists for this car
      if (polygon_publishers_.find(topic_name) == polygon_publishers_.end()) {
        polygon_publishers_[topic_name] =
          this->create_publisher<geometry_msgs::msg::PolygonStamped>(topic_name, 10);
        RCLCPP_DEBUG(this->get_logger(), "Created publisher for topic: %s", topic_name.c_str());
      }

      // Publish the polygon
      polygon_publishers_[topic_name]->publish(polygon_msg);

      car_id++;
    }
  }

  void carIdCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    for (const auto & marker : msg->markers) {
      RCLCPP_DEBUG(this->get_logger(), "Received car ID: %d", marker.id);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarPolygonPublisherNode>());
  rclcpp::shutdown();
  return 0;
}