#include "dbscan.hpp"
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Dense>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <set>
#include <unordered_map>
#include <vector>

class MarkerClusteringNode : public rclcpp::Node
{
public:
  // Constructor
  MarkerClusteringNode() : Node("marker_clustering_node")
  {
    // Subscribe to marker positions (as PoseArray)
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/all_marker_positions", 10,
      std::bind(&MarkerClusteringNode::poseCallback, this, std::placeholders::_1));

    // Publisher for estimated car orientations
    pose_array_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("/car_orientations", 10);

    // Publisher for visualization markers (car IDs) and hungarian algorithm
    publisher_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("/car_id", 10);

    RCLCPP_DEBUG(this->get_logger(), "Marker clustering node started.");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker;

  // Internal structure to store detected car components
  struct CarCandidate
  {
    Eigen::Vector2d orientation;
    int rear1, rear2, center;
    std::vector<int> front_markers;
  };

  double distance(const Eigen::Vector2d & a, const Eigen::Vector2d & b) { return (a - b).norm(); }

  Eigen::Vector2d midpoint(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
  {
    return (a + b) / 2.0;
  }

  Eigen::Vector2d direction(const Eigen::Vector2d & from, const Eigen::Vector2d & to)
  {
    Eigen::Vector2d dir = to - from;
    return dir.normalized();
  }

  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.stamp = this->get_clock()->now();
    pose_array_msg.header.frame_id = "map";

    visualization_msgs::msg::MarkerArray id_markers;

    std::vector<Eigen::Vector2d> points;
    for (const auto & pose : msg->poses) {
      points.emplace_back(pose.position.x, pose.position.y);
    }

    DBSCAN dbscan(0.25, 3);
    std::vector<int> labels = dbscan.fit(points);
    std::unordered_map<int, std::vector<Eigen::Vector2d>> clusters;
    // std::unordered_map<int, std::vector<int>> cluster_indices;

    for (size_t i = 0; i < labels.size(); ++i) {
      if (labels[i] >= 0) {
        clusters[labels[i]].push_back(points[i]);
        // cluster_indices[labels[i]].push_back(i);
      }
    }

    for (const auto & [cluster_id, cluster_points] : clusters) {
      // const std::vector<int>& indices = cluster_indices[cluster_id];
      size_t N = cluster_points.size();
      std::vector<bool> used(N, false);
      std::vector<CarCandidate> cars;

      for (size_t i = 0; i < N; ++i) {
        for (size_t j = i + 1; j < N; ++j) {
          if (used[i] || used[j]) continue;

          double rear_distance = distance(cluster_points[i], cluster_points[j]);
          if (std::abs(rear_distance - 0.05) > 0.01) continue;
          Eigen::Vector2d rear_center = midpoint(cluster_points[i], cluster_points[j]);
          Eigen::Vector2d dir = direction(cluster_points[i], cluster_points[j]);

          // Try two possible front directions (perpendicular to rear axis)
          Eigen::Vector2d front_dir_candi[2] = {
            Eigen::Vector2d(-dir.y(), dir.x()), Eigen::Vector2d(dir.y(), -dir.x())};

          // Expected position of front center marker (0.22m from rear center)
          Eigen::Vector2d expected_center_1 =
            rear_center + front_dir_candi[0] * 0.22;  // 0.22m front
          Eigen::Vector2d expected_center_2 =
            rear_center + front_dir_candi[1] * 0.22;  // 0.22m front

          int center_idx = -1;
          Eigen::Vector2d front_dir;

          for (size_t k = 0; k < N; ++k) {
            if (used[k] || k == i || k == j) continue;
            if (distance(cluster_points[k], expected_center_1) < 0.01) {
              center_idx = k;
              front_dir = front_dir_candi[0];
              break;
            } else if (distance(cluster_points[k], expected_center_2) < 0.01) {
              center_idx = k;
              front_dir = front_dir_candi[1];
              break;
            }
          }

          if (center_idx != -1) {
            double theta = std::atan2(front_dir.y(), front_dir.x());
            Eigen::Rotation2Dd rot(-theta);

            std::vector<int> front_markers;
            double length = 0.04;  // Rechteck vorne
            double width = 0.14;   // Rechteck vorne

            for (size_t idx = 0; idx < N; ++idx) {
              if (used[idx]) continue;
              Eigen::Vector2d rel = rot * (cluster_points[idx] - rear_center);
              if (
                rel.x() > (0.34 - length / 2.0) && rel.x() < (0.34 + length / 2.0) &&
                std::abs(rel.y()) < (width / 2.0 + 0.02)) {
                front_markers.push_back(idx);
              }
            }

            // mark used
            used[i] = true;
            used[j] = true;
            used[center_idx] = true;
            for (int f : front_markers) used[f] = true;

            cars.push_back(
              {front_dir, static_cast<int>(i), static_cast<int>(j), center_idx, front_markers});

            int total_markers = 2 + 1 + static_cast<int>(front_markers.size());
            RCLCPP_DEBUG(
              this->get_logger(), "Cluster %d: Detected car with %d markers (rear_center: %d,%d)",
              cluster_id, total_markers, rear_center.x(), rear_center.y());
          }
        }
      }

      bool all_marked = true;
      for (bool u : used) {
        if (!u) {
          all_marked = false;
          break;
        }
      }

      if (!all_marked) {
        RCLCPP_WARN(
          this->get_logger(), "Cluster %d has unassigned markers, possible mismatch.", cluster_id);
        break;
      } else {
        RCLCPP_DEBUG(
          this->get_logger(), "Cluster %d: Total detected cars: %ld", cluster_id, cars.size());

        for (const auto & car : cars) {
          geometry_msgs::msg::Pose pose;
          pose.position.x = cluster_points[car.center].x();
          pose.position.y = cluster_points[car.center].y();
          pose.position.z = 0.0;

          double theta = std::atan2(car.orientation.y(), car.orientation.x());
          tf2::Quaternion q;
          q.setRPY(0, 0, theta);  // Yaw only
          pose.orientation = tf2::toMsg(q);

          pose_array_msg.poses.push_back(pose);

          // id_markers
          visualization_msgs::msg::Marker id_marker;
          id_marker.header.frame_id = "gazebo_world";
          id_marker.header.stamp = this->get_clock()->now();
          id_marker.ns = "multi_int_display";
          id_marker.id = car.front_markers.size();

          id_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          id_marker.action = visualization_msgs::msg::Marker::ADD;

          id_marker.pose.position.x = cluster_points[car.center].x();
          id_marker.pose.position.y = cluster_points[car.center].y();
          id_marker.pose.position.z = 1.0;

          id_marker.pose.orientation.x = 0.0;
          id_marker.pose.orientation.y = 0.0;
          id_marker.pose.orientation.z = 0.0;
          id_marker.pose.orientation.w = 1.0;

          id_marker.text = std::to_string(id_marker.id);
          id_marker.scale.z = 0.5;
          id_marker.color.r = 1.0;
          id_marker.color.g = 1.0;
          id_marker.color.b = 0.0;
          id_marker.color.a = 1.0;
          id_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

          id_markers.markers.push_back(id_marker);
        }
      }
    }
    pose_array_publisher_->publish(pose_array_msg);
    publisher_marker->publish(id_markers);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerClusteringNode>());
  rclcpp::shutdown();
  return 0;
}
