// pick_place_demo.cpp
//
// Sends a scripted pick-and-place sequence of target poses to ik_node by
// publishing geometry_msgs/PoseStamped messages on "target_pose", with
// pauses between waypoints so you can watch the arm move in RViz2.
//
// C++ (rclcpp) port of robot_ik_pickplace/pick_place_demo.py.
//
// Sequence: home -> approach pick -> pick -> lift -> approach place -> place
// -> retreat -> home.
//
// NOTE: like ik_node.cpp, this was written against the rclcpp API but has
// not been build-verified in this environment (no ROS2 install here) --
// see ../CMakeLists.txt and ../package.xml for how it's meant to build
// inside a colcon workspace.

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_ik_pickplace_cpp/path_metrics.hpp"

using robot_ik_pickplace_cpp::path_length;
using robot_ik_pickplace_cpp::Waypoint;

namespace {

geometry_msgs::msg::PoseStamped make_pose(double x, double y, double z) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "base_link";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.w = 1.0;  // identity orientation for simplicity
  return msg;
}

// Mirrors WAYPOINTS in pick_place_demo.py exactly.
const std::vector<Waypoint> kWaypoints = {
    {"approach pick", 0.5, 0.3, 0.55},
    {"pick", 0.5, 0.3, 0.40},
    {"lift", 0.5, 0.3, 0.55},
    {"approach place", -0.4, 0.4, 0.45},
    {"place", -0.4, 0.4, 0.30},
    {"retreat", -0.4, 0.4, 0.45},
    {"home", 0.9, -0.2, 0.05},
};

}  // namespace

class PickPlaceDemo : public rclcpp::Node {
 public:
  PickPlaceDemo() : Node("pick_place_demo") {
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
  }

  void run_sequence(double dwell_seconds = 2.0) {
    RCLCPP_INFO(get_logger(), "Running %zu-waypoint sequence, %.3f m total "
                               "end-effector travel.",
                kWaypoints.size(), path_length(kWaypoints));

    for (const auto& wp : kWaypoints) {
      RCLCPP_INFO(get_logger(), "-> %s: (%.3f, %.3f, %.3f)", wp.label.c_str(),
                  wp.x, wp.y, wp.z);
      auto msg = make_pose(wp.x, wp.y, wp.z);
      msg.header.stamp = get_clock()->now();
      pub_->publish(msg);
      std::this_thread::sleep_for(
          std::chrono::duration<double>(dwell_seconds));
    }
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceDemo>();
  try {
    node->run_sequence();
  } catch (...) {
    // Mirror the Python version's KeyboardInterrupt handling: swallow and
    // shut down cleanly on interruption rather than crashing.
  }
  rclcpp::shutdown();
  return 0;
}
