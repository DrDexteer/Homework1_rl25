#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ArmControllerNode : public rclcpp::Node
{
  public:
    ArmControllerNode()
    : Node("arm_controller_node"), count_(0)
    {
      controller_type_ = this->declare_parameter<std::string>("controller_type", "position");

      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&ArmControllerNode::JointStatesCallback, this, _1));

      if (controller_type_ == "position") {
        position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
      } else if (controller_type_ == "trajectory") {
        trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
      } else {
        RCLCPP_FATAL(get_logger(), "Unknown controller_type='%s' (use 'position' or 'trajectory')", controller_type_.c_str());
        throw std::runtime_error("bad controller_type");
      }
      
      const size_t q = 4;  
      const size_t n_cmds = 10;
      
      for (size_t i = 1; i <= n_cmds; ++i) {
        const std::string pname = "cmd" + std::to_string(i);
        // default = zeros (4 giunti)
        auto cmd_i = this->declare_parameter<std::vector<double>>(pname, {0.0, 0.0, 0.0, 0.0});
        if (cmd_i.size() != q) {
          RCLCPP_FATAL(get_logger(), "Parameter '%s' must have %zu elements.",
                      pname.c_str(), q);
          throw std::runtime_error("bad command length");
        }
        commands_.push_back(std::move(cmd_i));
      }

      timer_ = this->create_wall_timer(5s, std::bind(&ArmControllerNode::joint_cmd_publish, this));
    }

  private:
    void JointStatesCallback(const sensor_msgs::msg::JointState & msg) const
    {
      std::string line = "q=[";
      for (size_t i = 0; i < msg.position.size(); ++i) {
        line += std::to_string(msg.position[i]);
        if (i + 1 < msg.position.size()) line += ", ";
      }
      line += "]";
      //RCLCPP_INFO(this->get_logger(), "%s", line.c_str());
    }

    void joint_cmd_publish()
    {
      if (commands_.empty()) return;
      const size_t idx = std::min(count_, commands_.size() - 1);
      const auto &q = commands_[idx];

      if (controller_type_ == "position") {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = q;
        RCLCPP_INFO(this->get_logger(),
          "[position] cmd #%zu -> [%f %f %f %f]", idx, q[0], q[1], q[2], q[3]);
        position_publisher_->publish(msg);
      } else {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = {"j0","j1","j2","j3"};

        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = q;
        p.time_from_start = rclcpp::Duration(2,0); // reach target in 2s

        traj.points.push_back(p);
        RCLCPP_INFO(get_logger(),
          "[trajectory] point #%zu @2s -> [%f %f %f %f]", idx, q[0], q[1], q[2], q[3]);
        trajectory_publisher_->publish(traj);

      }
      ++count_;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
    std::vector<std::vector<double>> commands_;
    size_t count_;
    std::string controller_type_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControllerNode>());
  rclcpp::shutdown();
  return 0;
}