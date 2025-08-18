#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

// Hungarian Algorithm for assignment
class Hungarian
{
public:
  static std::vector<int> Solve(const std::vector<std::vector<double>> & cost)
  {
    int n = cost.size();
    int m = cost[0].size();
    std::vector<double> u(n + 1), v(m + 1);
    std::vector<int> p(m + 1), way(m + 1);

    for (int i = 1; i <= n; ++i) {
      p[0] = i;
      int j0 = 0;
      std::vector<double> minv(m + 1, std::numeric_limits<double>::max());
      std::vector<bool> used(m + 1, false);
      do {
        used[j0] = true;
        int i0 = p[j0], j1;
        double delta = std::numeric_limits<double>::max();
        for (int j = 1; j <= m; ++j) {
          if (!used[j]) {
            double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
            if (cur < minv[j]) {
              minv[j] = cur;
              way[j] = j0;
            }
            if (minv[j] < delta) {
              delta = minv[j];
              j1 = j;
            }
          }
        }
        for (int j = 0; j <= m; ++j) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }
        j0 = j1;
      } while (p[j0] != 0);
      do {
        int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0);
    }

    std::vector<int> ans(n, -1);
    for (int j = 1; j <= m; ++j) {
      if (p[j] > 0 && p[j] <= n) ans[p[j] - 1] = j - 1;
    }
    return ans;
  }
};

struct Car
{
  int track_id;
  geometry_msgs::msg::Point position;
  double yaw;
  bool visible;
};

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode() : Node("tracker_node")
  {
    sub1_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot1/global_pose", 10,
      std::bind(&TrackerNode::robot1Callback, this, std::placeholders::_1));

    sub2_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot2/global_pose", 10,
      std::bind(&TrackerNode::robot2Callback, this, std::placeholders::_1));

    sub3_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot3/global_pose", 10,
      std::bind(&TrackerNode::robot3Callback, this, std::placeholders::_1));

    sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/car_id", 10, std::bind(&TrackerNode::callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracked_cars", 10);

    RCLCPP_DEBUG(this->get_logger(), "TrackerNode started.");
  }

private:
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unordered_map<
    int, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
    car_publishers_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub1_, sub2_, sub3_;

  std::array<geometry_msgs::msg::Point, 3> poses_;
  std::array<double, 3> yaws_;
  std::array<bool, 3> received_ = {false, false, false};

  std::vector<Car> tracked_cars_;
  bool initialized_ = false;
  const double yaw_weight_ = 0.5;

  double getYaw(const geometry_msgs::msg::Quaternion & q) const
  {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  double costFunction(const Car & prev, const geometry_msgs::msg::Point & pos, double yaw) const
  {
    double dx = prev.position.x - pos.x;
    double dy = prev.position.y - pos.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double dyaw = std::fabs(prev.yaw - yaw);
    if (dyaw > M_PI) dyaw = 2 * M_PI - dyaw;
    return dist + yaw_weight_ * dyaw;
  }

  void initializeTracks(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    tracked_cars_.clear();
    for (const auto & marker : msg->markers) {
      Car car;
      car.track_id = marker.id;
      car.position = marker.pose.position;
      car.yaw = getYaw(marker.pose.orientation);
      car.visible = true;
      tracked_cars_.push_back(car);
    }
    initialized_ = true;
  }

  void updateTracks(
    const std::vector<geometry_msgs::msg::Point> & positions, const std::vector<double> & yaws)
  {
    int N = static_cast<int>(tracked_cars_.size());
    int M = static_cast<int>(positions.size());

    std::vector<std::vector<double>> cost(N, std::vector<double>(M));
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < M; ++j)
        cost[i][j] = costFunction(tracked_cars_[i], positions[j], yaws[j]);

    std::vector<int> assignment = Hungarian::Solve(cost);

    for (auto & car : tracked_cars_) car.visible = false;

    for (int i = 0; i < N; ++i) {
      int j = assignment[i];
      if (j >= 0 && j < M) {
        tracked_cars_[i].position = positions[j];
        tracked_cars_[i].yaw = yaws[j];
        tracked_cars_[i].visible = true;
      }
    }
  }

  void callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "callback started");

    if (!initialized_ && msg->markers.size() == 3) {
      initializeTracks(msg);
    } else if (initialized_) {
      // if (received_[0] && received_[1] && received_[2]) {
      //   std::vector<geometry_msgs::msg::Point> pos_vec(poses_.begin(), poses_.end());
      //   std::vector<double> yaw_vec(yaws_.begin(), yaws_.end());
      //   updateTracks(pos_vec, yaw_vec);
      //   received_[0] = false;
      //   received_[1] = false;
      //   received_[2] = false;
      // }
      std::vector<geometry_msgs::msg::Point> positions;
      std::vector<double> yaws;
      positions.reserve(msg->markers.size());
      yaws.reserve(msg->markers.size());

      for (const auto & marker : msg->markers) {
        positions.push_back(marker.pose.position);
        yaws.push_back(getYaw(marker.pose.orientation));
      }
      updateTracks(positions, yaws);
    }

    RCLCPP_DEBUG(this->get_logger(), "Tracked cars:");
    for (const auto & car : tracked_cars_) {
      RCLCPP_DEBUG(
        this->get_logger(), "ID: %d, Position: (%.2f, %.2f, %.2f), Yaw: %.2f, Visible: %s",
        car.track_id, car.position.x, car.position.y, car.position.z, car.yaw,
        car.visible ? "true" : "false");
    }

    publishMarkers();
    publishCarTopics();
  }

  void robot1Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    poses_[0] = msg->pose.position;
    yaws_[0] = getYaw(msg->pose.orientation);
    received_[0] = true;
    if (initialized_) {
      if (received_[0] && received_[1] && received_[2]) {
        std::vector<geometry_msgs::msg::Point> pos_vec(poses_.begin(), poses_.end());
        std::vector<double> yaw_vec(yaws_.begin(), yaws_.end());
        updateTracks(pos_vec, yaw_vec);
        received_[0] = false;
        received_[1] = false;
        received_[2] = false;
      }
    }
  }

  void robot2Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    poses_[1] = msg->pose.position;
    yaws_[1] = getYaw(msg->pose.orientation);
    received_[1] = true;
  }

  void robot3Callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    poses_[2] = msg->pose.position;
    yaws_[2] = getYaw(msg->pose.orientation);
    received_[2] = true;
  }

  void publishCarTopics()
  {
    for (const auto & car : tracked_cars_) {
      // Create a unique topic name for each car based on its ID
      std::string topic_name = "/robot" + std::to_string(car.track_id) + "/mot_pose";

      // Check if a publisher already exists for this car
      if (car_publishers_.find(car.track_id) == car_publishers_.end()) {
        car_publishers_[car.track_id] =
          this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name, 10);
        RCLCPP_DEBUG(this->get_logger(), "Created publisher for topic: %s", topic_name.c_str());
      }

      // Construct PoseWithCovarianceStamped message
      geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
      pose_cov_msg.header.stamp = this->get_clock()->now();
      pose_cov_msg.header.frame_id = "map";  // or "odom", depending on your frame usage

      pose_cov_msg.pose.pose.position = car.position;

      tf2::Quaternion q;
      q.setRPY(0, 0, car.yaw);
      pose_cov_msg.pose.pose.orientation.x = q.x();
      pose_cov_msg.pose.pose.orientation.y = q.y();
      pose_cov_msg.pose.pose.orientation.z = q.z();
      pose_cov_msg.pose.pose.orientation.w = q.w();
      // Set covariance values
      pose_cov_msg.pose.covariance = {0.01, 0, 0,   0, 0,   0, 0, 0.01, 0, 0,   0, 0,
                                      0,    0, 1e6, 0, 0,   0, 0, 0,    0, 1e6, 0, 0,
                                      0,    0, 0,   0, 1e6, 0, 0, 0,    0, 0,   0, 0.01};

      // Publish the message
      car_publishers_[car.track_id]->publish(pose_cov_msg);
    }
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto & car : tracked_cars_) {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = "map";
      arrow.header.stamp = this->get_clock()->now();
      arrow.ns = "car_arrows";
      arrow.id = car.track_id;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.position = car.position;

      tf2::Quaternion q;
      q.setRPY(0, 0, car.yaw);
      arrow.pose.orientation.x = q.x();
      arrow.pose.orientation.y = q.y();
      arrow.pose.orientation.z = q.z();
      arrow.pose.orientation.w = q.w();

      arrow.scale.x = 1.0;
      arrow.scale.y = 0.2;
      arrow.scale.z = 0.2;

      arrow.color.r = car.visible ? 0.0f : 1.0f;
      arrow.color.g = 1.0f;
      arrow.color.b = 0.0f;
      arrow.color.a = 1.0f;

      visualization_msgs::msg::Marker text;
      text.header = arrow.header;
      text.ns = "car_labels";
      text.id = car.track_id + 1000;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = car.position;
      text.pose.position.z += 1.0;
      text.scale.z = 0.5;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text = "ID: " + std::to_string(car.track_id);

      marker_array.markers.push_back(arrow);
      marker_array.markers.push_back(text);
    }

    marker_pub_->publish(marker_array);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
