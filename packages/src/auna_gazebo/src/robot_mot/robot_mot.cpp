#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <vector>
#include <cmath>
#include <climits>
#include <unordered_map>

// ========================== 匈牙利算法实现 ==========================
class Hungarian {
public:
    static std::vector<int> Solve(const std::vector<std::vector<double>>& cost) {
        int n = cost.size();
        int m = cost[0].size();
        std::vector<int> u(n+1), v(m+1), p(m+1), way(m+1);
        for (int i = 1; i <= n; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<int> minv(m+1, INT_MAX);
            std::vector<bool> used(m+1, false);
            do {
                used[j0] = true;
                int i0 = p[j0], delta = INT_MAX, j1;
                for (int j = 1; j <= m; ++j) {
                    if (!used[j]) {
                        int cur = cost[i0-1][j-1] - u[i0] - v[j];
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
            if (p[j] > 0 && p[j] <= n)
                ans[p[j]-1] = j-1;
        }
        return ans;
    }
};

// ========================== Tracker 节点 ==========================

struct Car {
    int id;
    geometry_msgs::msg::Point position;
    double yaw;
};

class TrackerNode : public rclcpp::Node {
public:
    TrackerNode() : Node("tracker_node") {
        sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/car_id", 10,
            std::bind(&TrackerNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "TrackerNode started.");
    }

private:
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
    std::vector<Car> tracked_cars_;
    int next_id_ = 1;
    double yaw_weight_ = 0.5;

    double getYaw(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    double costFunction(const Car& prev, const Car& curr) {
        double dx = prev.position.x - curr.position.x;
        double dy = prev.position.y - curr.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        double dyaw = std::fabs(prev.yaw - curr.yaw);
        if (dyaw > M_PI) dyaw = 2 * M_PI - dyaw;

        return distance + yaw_weight_ * dyaw;
    }

    void callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        std::vector<Car> current_cars;

        for (const auto& marker : msg->markers) {
            Car c;
            c.position = marker.pose.position;
            c.yaw = getYaw(marker.pose.orientation);
            c.id = -1;
            current_cars.push_back(c);
        }

        if (tracked_cars_.empty()) {
            for (auto& c : current_cars) {
                c.id = next_id_++;
                tracked_cars_.push_back(c);
                RCLCPP_INFO(this->get_logger(), "Init car ID %d", c.id);
            }
            return;
        }

        // 构建 cost 矩阵
        int N = tracked_cars_.size();
        int M = current_cars.size();
        std::vector<std::vector<double>> cost(N, std::vector<double>(M, 0));

        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                cost[i][j] = costFunction(tracked_cars_[i], current_cars[j]);

        // 使用匈牙利算法求解
        std::vector<int> assignment = Hungarian::Solve(cost);

        std::vector<Car> new_tracked;

        std::vector<bool> matched(M, false);
        for (size_t i = 0; i < assignment.size(); ++i) {
            int j = assignment[i];
            if (j >= 0 && j < M) {
                current_cars[j].id = tracked_cars_[i].id;
                new_tracked.push_back(current_cars[j]);
                matched[j] = true;
            }
        }

        // 分配新 ID 给未匹配的
        for (size_t j = 0; j < M; ++j) {
            if (!matched[j]) {
                current_cars[j].id = next_id_++;
                new_tracked.push_back(current_cars[j]);
                RCLCPP_INFO(this->get_logger(), "New car ID %d", current_cars[j].id);
            }
        }

        tracked_cars_ = new_tracked;
        RCLCPP_INFO(this->get_logger(), "== Tracked Cars (Frame Update) ==");

        for (const auto& car : tracked_cars_) {
            RCLCPP_INFO(this->get_logger(),
                "Car ID: %d | Pos: [%.2f, %.2f] | Yaw: %.2f deg",
                car.id,
                car.position.x,
                car.position.y,
                car.yaw * 180.0 / M_PI  // 弧度转角度
            );
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackerNode>());
    rclcpp::shutdown();
    return 0;
}
