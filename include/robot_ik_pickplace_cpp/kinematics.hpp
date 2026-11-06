// kinematics.hpp
//
// From-scratch forward and inverse kinematics for a 6-DOF revolute arm,
// using standard Denavit-Hartenberg (DH) parameters.
//
// C++ port of robot_ik_pickplace/kinematics.py. Pure C++/Eigen, no ROS2
// dependency on purpose: same philosophy as the Python original -- keep
// the math testable and debuggable in isolation (see standalone/ and
// test/) before it's wired into a ROS2 node (see src/ik_node.cpp).
//
// DH convention used (standard, not "modified"):
//     A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)

#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace robot_ik_pickplace_cpp {

// One row of a Denavit-Hartenberg parameter table.
struct DHParam {
  double theta_offset = 0.0;  // fixed offset added to the joint variable (rad)
  double d = 0.0;              // link offset along previous z (m)
  double a = 0.0;              // link length along new x (m)
  double alpha = 0.0;          // link twist about new x (rad)
};

using JointLimits = std::vector<std::pair<double, double>>;

// Example 6-DOF anthropomorphic arm with a spherical wrist. Lengths are
// illustrative -- swap these for a real robot's datasheet values (e.g. a
// UR5) once you're ready to match a specific URDF exactly. Mirrors
// DEFAULT_DH_TABLE in kinematics.py exactly.
std::vector<DHParam> default_dh_table();

// radians; tighten to match a real robot. Mirrors JOINT_LIMITS in
// kinematics.py.
JointLimits default_joint_limits();

// Homogeneous transform for one DH row.
Eigen::Matrix4d dh_transform(double theta, double d, double a, double alpha);

// Result of a numerical inverse-kinematics solve.
struct IKResult {
  Eigen::VectorXd joint_angles;
  bool converged = false;
  int iterations = 0;
};

// Forward/inverse kinematics for a 6-revolute-joint serial arm.
class SixAxisArm {
 public:
  explicit SixAxisArm(std::vector<DHParam> dh_table = default_dh_table(),
                       JointLimits joint_limits = default_joint_limits());

  int n_joints() const { return n_joints_; }
  const std::vector<DHParam>& dh_table() const { return dh_table_; }
  const JointLimits& joint_limits() const { return joint_limits_; }

  // ---------- Forward kinematics ----------

  // Returns the list of cumulative homogeneous transforms T_0_1 .. T_0_n,
  // i.e. the last entry is the end-effector pose in the base frame. Also
  // useful for plotting each joint's position along the chain.
  std::vector<Eigen::Matrix4d> forward_kinematics(
      const Eigen::VectorXd& joint_angles) const;

  Eigen::Matrix4d end_effector_pose(const Eigen::VectorXd& joint_angles) const;

  // Cartesian xyz of each joint origin, including the base at row 0.
  // Returns an (n_joints + 1) x 3 matrix, one row per joint origin.
  Eigen::MatrixXd joint_positions(const Eigen::VectorXd& joint_angles) const;

  // ---------- Jacobian ----------

  // 6xN geometric Jacobian (linear velocity rows 0-2, angular rows 3-5)
  // for revolute joints, computed directly from the FK chain:
  //     Jv_i = z_(i-1) x (o_n - o_(i-1))
  //     Jw_i = z_(i-1)
  Eigen::MatrixXd geometric_jacobian(const Eigen::VectorXd& joint_angles) const;

  // ---------- Inverse kinematics ----------

  // Damped least-squares (Levenberg-Marquardt style) numerical IK.
  //
  // This avoids the singularity blow-ups of a plain pseudo-inverse
  // Jacobian solver, at the cost of needing a damping term. It is a
  // standard, well-documented approach and a reasonable starting point
  // before you optionally add an analytical solver for a specific robot
  // geometry.
  IKResult inverse_kinematics(
      const Eigen::Matrix4d& target_pose,
      std::optional<Eigen::VectorXd> initial_guess = std::nullopt,
      int max_iters = 200, double tol = 1e-4, double damping = 0.05) const;

 private:
  // Wrap each joint angle into (-pi, pi] before clamping to its limits.
  //
  // The damped-least-squares loop above has no bound on joint angle
  // magnitude -- since forward kinematics is 2*pi-periodic in each joint,
  // it can (and does) converge at raw angles like 8.02 rad, which is the
  // same arm configuration as ~1.74 rad. A plain clamp of that raw value
  // to [-pi, pi] would snap it to +pi instead of wrapping it, silently
  // producing a *different* configuration than the one that was just
  // verified to match the target pose. Wrapping with modulo arithmetic
  // first preserves the actual configuration; the subsequent clamp then
  // only does real work for joints whose limits are tighter than a full
  // +/-pi range.
  Eigen::VectorXd clip_to_limits(const Eigen::VectorXd& q) const;

  // 6-vector error: 3 position + 3 orientation (axis-angle approx).
  static Eigen::VectorXd pose_error(const Eigen::Matrix4d& current,
                                     const Eigen::Matrix4d& target);

  std::vector<DHParam> dh_table_;
  JointLimits joint_limits_;
  int n_joints_;
};

// Convenience: build a 4x4 pose with identity (or given) orientation.
Eigen::Matrix4d pose_from_xyz(double x, double y, double z,
                               const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity());

}  // namespace robot_ik_pickplace_cpp
