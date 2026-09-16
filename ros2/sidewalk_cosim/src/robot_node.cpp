// SPDX-License-Identifier: GPL-2.0-only
//
// One cooperative robot: random-waypoint motion in a square area, publishes its
// pose every period_ms on /robot_<id>/pose and listens to what its neighbours'
// state messages delivered over the (simulated) sidelink on /robot_<id>/neighbors.

#include <chrono>
#include <cmath>
#include <random>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RobotNode : public rclcpp::Node
{
  public:
    RobotNode() : Node("robot")
    {
        id_ = declare_parameter<int>("id", 0);
        area_ = declare_parameter<double>("area_size", 50.0);
        speed_ = declare_parameter<double>("speed", 1.5);
        const int period_ms = declare_parameter<int>("period_ms", 100);
        rng_.seed(1000 + id_);

        std::uniform_real_distribution<double> u(0.0, area_);
        x_ = u(rng_);
        y_ = u(rng_);
        pickWaypoint();

        const std::string ns = "/robot_" + std::to_string(id_);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(ns + "/pose", 10);
        neigh_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            ns + "/neighbors", 10,
            [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
                neighbours_seen_ += msg->poses.size();
            });
        last_ = now();
        timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this] { tick(); });
        report_ = create_wall_timer(5s, [this] {
            RCLCPP_INFO(get_logger(), "robot %d at (%.1f, %.1f), %zu neighbour states received",
                        id_, x_, y_, neighbours_seen_);
        });
    }

  private:
    void pickWaypoint()
    {
        std::uniform_real_distribution<double> u(0.0, area_);
        wx_ = u(rng_);
        wy_ = u(rng_);
    }

    void tick()
    {
        const auto t = now();
        const double dt = (t - last_).seconds();
        last_ = t;
        // move towards the waypoint at constant speed; pick a new one on arrival
        const double dx = wx_ - x_, dy = wy_ - y_, d = std::hypot(dx, dy);
        const double step = speed_ * dt;
        if (d <= step)
        {
            x_ = wx_; y_ = wy_;
            pickWaypoint();
        }
        else
        {
            x_ += dx / d * step;
            y_ += dy / d * step;
        }
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = t;
        p.header.frame_id = "map";
        p.pose.position.x = x_;
        p.pose.position.y = y_;
        p.pose.position.z = 0.5;
        p.pose.orientation.w = 1.0;
        pose_pub_->publish(p);
    }

    int id_;
    double area_, speed_, x_, y_, wx_, wy_;
    std::mt19937 rng_;
    size_t neighbours_seen_{0};
    rclcpp::Time last_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr neigh_sub_;
    rclcpp::TimerBase::SharedPtr timer_, report_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}
