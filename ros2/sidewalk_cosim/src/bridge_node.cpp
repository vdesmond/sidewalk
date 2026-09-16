// SPDX-License-Identifier: GPL-2.0-only
//
// Co-simulation bridge between ROS 2 robots and the ns-3 / 5G-LENA sidelink model.
//
// Acts as the "server" of the ns3-cosim Gateway protocol: ns-3 (sidewalk-robots
// --cosimPort=<port>) connects over TCP and from then on this node owns simulated
// time. Every step_ms it sends
//     "sec nsec  x y z send  x y z send ...\r\n"        (one x y z send per robot)
// and blocks until ns-3 replies with
//     "deliv_0 deliv_1 ... deliv_{N-1}\r\n"              deliv_j = "src:latency_us;src:latency_us;..."
// The poses come from /robot_i/pose; 'send' is 1 when robot i published a new pose
// since the previous step (one state message per new pose). Every delivery is
// republished as the sender's pose (as of its transmission) on /robot_j/neighbors
// and its latency on /sidewalk/latency_ms, and appended to a CSV.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class BridgeNode : public rclcpp::Node
{
  public:
    BridgeNode() : Node("sidewalk_bridge")
    {
        n_ = declare_parameter<int>("num_robots", 10);
        port_ = declare_parameter<int>("port", 8000);
        step_ms_ = declare_parameter<int>("step_ms", 10);
        duration_s_ = declare_parameter<double>("duration_s", 0.0); // 0 = until shutdown
        realtime_ = declare_parameter<bool>("realtime", true);       // pace steps to wall clock
        // ns-3 activates the sidelink bearers at 2 s and RobotGateway ignores sends before
        // 2.1 s; PRR counts only messages *sent* from warmup_s on (numerator and denominator)
        warmup_s_ = declare_parameter<double>("warmup_s", 2.1);
        csv_path_ = declare_parameter<std::string>("csv_path", "/tmp/sidewalk-cosim.csv");

        robots_.resize(n_);
        for (int i = 0; i < n_; i++)
        {
            const std::string ns = "/robot_" + std::to_string(i);
            robots_[i].pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
                ns + "/pose", 10,
                [this, i](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                    std::lock_guard<std::mutex> lk(mu_);
                    robots_[i].pose = msg->pose;
                    robots_[i].fresh = true;
                    robots_[i].have_pose = true;
                });
            robots_[i].neigh_pub =
                create_publisher<geometry_msgs::msg::PoseArray>(ns + "/neighbors", 10);
        }
        latency_pub_ = create_publisher<std_msgs::msg::Float64>("/sidewalk/latency_ms", 100);
        csv_.open(csv_path_);
        csv_ << "sim_time_s,src,dst,latency_us\n";
        worker_ = std::thread([this] { run(); });
    }

    ~BridgeNode() override
    {
        stop_ = true;
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

  private:
    struct Robot
    {
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr neigh_pub;
        geometry_msgs::msg::Pose pose;      // latest pose from the robot
        geometry_msgs::msg::Pose sent_pose; // pose carried by its last state message
        bool fresh{false}, have_pose{false};
    };

    int acceptClient()
    {
        int srv = socket(AF_INET, SOCK_STREAM, 0);
        int reuse = 1;
        setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 || listen(srv, 1) < 0)
        {
            RCLCPP_FATAL(get_logger(), "cannot listen on port %d", port_);
            return -1;
        }
        RCLCPP_INFO(get_logger(), "waiting for ns-3 on port %d ...", port_);
        // poll accept so Ctrl-C still works while waiting
        timeval tv{1, 0};
        setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        int cli = -1;
        while (!stop_ && rclcpp::ok() && (cli = accept(srv, nullptr, nullptr)) < 0)
        {
        }
        close(srv);
        if (cli >= 0)
        {
            RCLCPP_INFO(get_logger(), "ns-3 connected");
        }
        return cli;
    }

    bool recvLine(int fd, std::string& line)
    {
        char buf[65536];
        while (rx_buf_.find("\r\n") == std::string::npos)
        {
            const ssize_t n = recv(fd, buf, sizeof(buf), 0);
            if (n <= 0)
            {
                return false;
            }
            rx_buf_.append(buf, n);
        }
        const size_t end = rx_buf_.find("\r\n");
        line = rx_buf_.substr(0, end);
        rx_buf_.erase(0, end + 2);
        return true;
    }

    void run()
    {
        const int fd = acceptClient();
        if (fd < 0)
        {
            return;
        }
        const auto step = std::chrono::milliseconds(step_ms_);
        auto next_wall = std::chrono::steady_clock::now();
        int64_t sim_ns = 0;
        std::string line;

        while (!stop_ && rclcpp::ok() &&
               (duration_s_ <= 0 || sim_ns < static_cast<int64_t>(duration_s_ * 1e9)))
        {
            // ---- build the step message ----
            std::ostringstream out;
            out << sim_ns / 1000000000 << ' ' << sim_ns % 1000000000;
            {
                std::lock_guard<std::mutex> lk(mu_);
                for (auto& r : robots_)
                {
                    const bool send = r.fresh && r.have_pose;
                    if (send)
                    {
                        r.sent_pose = r.pose;
                    }
                    r.fresh = false;
                    out << ' ' << r.pose.position.x << ' ' << r.pose.position.y << ' '
                        << r.pose.position.z << ' ' << (send ? 1 : 0);
                    sent_ += send && sim_ns >= static_cast<int64_t>(warmup_s_ * 1e9);
                }
            }
            out << "\r\n";
            const std::string msg = out.str();
            if (::send(fd, msg.data(), msg.size(), 0) < 0 || !recvLine(fd, line))
            {
                RCLCPP_WARN(get_logger(), "ns-3 disconnected");
                break;
            }
            handleReply(line, sim_ns);

            sim_ns += step_ms_ * 1000000LL;
            if (realtime_)
            {
                next_wall += step;
                std::this_thread::sleep_until(next_wall);
            }
            if (sim_ns % 5000000000LL == 0)
            {
                report(sim_ns);
            }
        }
        const std::string bye = "-1 0\r\n";
        ::send(fd, bye.data(), bye.size(), 0);
        close(fd);
        report(sim_ns);
        csv_.close();
        RCLCPP_INFO(get_logger(), "done; deliveries written to %s", csv_path_.c_str());
        rclcpp::shutdown();
    }

    void handleReply(const std::string& line, int64_t sim_ns)
    {
        // N fields separated by single spaces; empty fields are legal
        std::vector<std::string> fields;
        size_t start = 0;
        for (size_t i = 0; i <= line.size(); i++)
        {
            if (i == line.size() || line[i] == ' ')
            {
                fields.push_back(line.substr(start, i - start));
                start = i + 1;
            }
        }
        if (static_cast<int>(fields.size()) != n_)
        {
            RCLCPP_WARN_ONCE(get_logger(), "reply has %zu fields, expected %d", fields.size(), n_);
        }
        std::lock_guard<std::mutex> lk(mu_);
        for (int j = 0; j < n_ && j < static_cast<int>(fields.size()); j++)
        {
            if (fields[j].empty())
            {
                continue;
            }
            geometry_msgs::msg::PoseArray neigh;
            neigh.header.stamp = now();
            neigh.header.frame_id = "map";
            std::stringstream ss(fields[j]);
            std::string item;
            while (std::getline(ss, item, ';'))
            {
                const size_t c = item.find(':');
                const int src = std::stoi(item.substr(0, c));
                const int64_t lat_us = std::stoll(item.substr(c + 1));
                if (src >= 0 && src < n_)
                {
                    neigh.poses.push_back(robots_[src].sent_pose);
                }
                std_msgs::msg::Float64 l;
                l.data = lat_us / 1000.0;
                latency_pub_->publish(l);
                csv_ << sim_ns / 1e9 << ',' << src << ',' << j << ',' << lat_us << '\n';
                if (sim_ns - lat_us * 1000 >= static_cast<int64_t>(warmup_s_ * 1e9))
                {
                    latencies_us_.push_back(lat_us);
                }
            }
            robots_[j].neigh_pub->publish(neigh);
        }
    }

    void report(int64_t sim_ns)
    {
        if (latencies_us_.empty())
        {
            RCLCPP_INFO(get_logger(), "t=%.1fs sent=%zu delivered=0", sim_ns / 1e9, sent_);
            return;
        }
        std::vector<int64_t> l = latencies_us_;
        std::sort(l.begin(), l.end());
        const double prr = n_ > 1 ? double(l.size()) / (double(sent_) * (n_ - 1)) : 0.0;
        RCLCPP_INFO(get_logger(),
                    "t=%.1fs sent=%zu delivered=%zu PRR=%.3f latency p50=%.1f p95=%.1f ms",
                    sim_ns / 1e9, sent_, l.size(), prr, l[l.size() / 2] / 1e3,
                    l[static_cast<size_t>(0.95 * (l.size() - 1))] / 1e3);
    }

    int n_, port_, step_ms_;
    double duration_s_, warmup_s_;
    bool realtime_;
    std::string csv_path_, rx_buf_;
    std::vector<Robot> robots_;
    std::mutex mu_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
    std::ofstream csv_;
    size_t sent_{0};
    std::vector<int64_t> latencies_us_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgeNode>());
    rclcpp::shutdown();
    return 0;
}
