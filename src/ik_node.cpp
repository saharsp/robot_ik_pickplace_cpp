// ik_node.cpp
//
// ROS2 node that wraps the from-scratch SixAxisArm kinematics from
// kinematics.hpp -- C++ (rclcpp) port of robot_ik_pickplace/ik_node.py.
//
//   - Subscribes to geometry_msgs/PoseStamped on "target_pose".
//   - Solves IK for the requested pose (seeded from the current joint
//     state, so motion is continuous rather than jumping to a fresh
//     solution).
//   - Publishes sensor_msgs/JointState on "joint_states" so
//     robot_state_publisher can broadcast TF and RViz2 can visualize the
//     arm moving.
//
// This is intentionally a small, readable node -- the IK math itself lives
// in kinematics.hpp/.cpp so it stays testable without ROS2 (see
// standalone/ and test/).
//
// NOTE: this file was written against the rclcpp/ROS2 message APIs but has
// NOT been build-verified in this environment (no ROS2 install here). It
// needs to be compiled inside a colcon workspace with rclcpp, sensor_msgs,
// geometry_msgs, and visualization_msgs available (see ../CMakeLists.txt
// and ../package.xml) before trusting it on real hardware or in RViz2.

#include <array>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "robot_ik_pickplace_cpp/kinematics.hpp"

namespace {

using robot_ik_pickplace_cpp::SixAxisArm;

const std::array<std::string, 6> kJointNames = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

// Convert a geometry_msgs/Pose into a 4x4 homogeneous transform.
Eigen::Matrix4d pose_msg_to_matrix(const geometry_msgs::msg::Pose& pose_msg) {
  const auto& p = pose_msg.position;
  const auto& q = pose_msg.orientation;
  const double x = q.x, y = q.y, z = q.z, w = q.w;

  Eigen::Matrix3d R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
       2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
       2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T(0, 3) = p.x;
  T(1, 3) = p.y;
  T(2, 3) = p.z;
  return T;
}

}  // namespace

class IKNode : public rclcpp::Node {
 public:
  IKNode() : Node("ik_node"), current_q_(Eigen::VectorXd::Zero(arm_.n_joints())) {
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("target_marker", 10);
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "target_pose", 10,
        std::bind(&IKNode::on_target_pose, this, std::placeholders::_1));

    // Publish the current joint state at a steady rate so RViz2/TF stay live.
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                std::bind(&IKNode::publish_joint_state, this));

    RCLCPP_INFO(get_logger(),
                "ik_node ready. Publish a geometry_msgs/PoseStamped on "
                "'target_pose' to move the arm.");
  }

 private:
  void on_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    publish_target_marker(*msg);

    Eigen::Matrix4d target = pose_msg_to_matrix(msg->pose);
    auto result = arm_.inverse_kinematics(target, current_q_);

    Eigen::Matrix4d achieved = arm_.end_effector_pose(result.joint_angles);
    const double pos_error =
        (achieved.block<3, 1>(0, 3) - target.block<3, 1>(0, 3)).norm();

    if (!result.converged) {
      RCLCPP_WARN(get_logger(),
                  "IK did not converge in %d iterations for requested pose "
                  "(%.3f, %.3f, %.3f); using best-effort solution (position "
                  "error %.4f m).",
                  result.iterations, msg->pose.position.x, msg->pose.position.y,
                  msg->pose.position.z, pos_error);
    } else if (pos_error > 1e-3) {
      // Converged on paper but the solution that actually got applied
      // (after limit clipping) no longer matches the target -- this is the
      // exact failure mode a naive clamp on an unwrapped angle can cause.
      // Flag it loudly instead of letting it pass as success.
      RCLCPP_ERROR(get_logger(),
                   "IK reported convergence in %d iterations for target "
                   "(%.3f, %.3f, %.3f) but the applied solution misses it by "
                   "%.4f m -- check joint limit clipping.",
                   result.iterations, msg->pose.position.x, msg->pose.position.y,
                   msg->pose.position.z, pos_error);
    } else {
      RCLCPP_INFO(get_logger(),
                  "IK converged in %d iterations for target (%.3f, %.3f, "
                  "%.3f).",
                  result.iterations, msg->pose.position.x, msg->pose.position.y,
                  msg->pose.position.z);
    }

    current_q_ = result.joint_angles;
  }

  // Small sphere at the commanded target, so RViz2 shows at a glance
  // whether the arm's actual end-effector (see TF: tool0) lands on it.
  void publish_target_marker(const geometry_msgs::msg::PoseStamped& msg) {
    visualization_msgs::msg::Marker marker;
    marker.header = msg.header;
    marker.ns = "target_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = msg.pose;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.06;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.9;
    marker_pub_->publish(marker);
  }

  void publish_joint_state() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name.assign(kJointNames.begin(), kJointNames.end());
    msg.position.assign(current_q_.data(), current_q_.data() + current_q_.size());
    joint_pub_->publish(msg);
  }

  SixAxisArm arm_;
  Eigen::VectorXd current_q_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();
  return 0;
}
