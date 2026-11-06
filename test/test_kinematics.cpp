// test_kinematics.cpp
//
// Catch2 port of test/test_kinematics.py -- same six cases as the Python
// suite, including the limit-wrapping regression test.

#include <Eigen/Dense>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "robot_ik_pickplace_cpp/kinematics.hpp"

using robot_ik_pickplace_cpp::pose_from_xyz;
using robot_ik_pickplace_cpp::SixAxisArm;
using Catch::Approx;

namespace {

// Eigen's isApprox() divides by the operands' norms, which misbehaves for
// matrices that are legitimately close to zero (e.g. a translation column
// of all zeros). Use a plain elementwise absolute-tolerance comparison
// instead, matching numpy.allclose's default atol/rtol behavior closely
// enough for these tests.
bool all_close(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                double atol = 1e-8, double rtol = 1e-5) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  for (int i = 0; i < a.rows(); ++i) {
    for (int j = 0; j < a.cols(); ++j) {
      const double diff = std::abs(a(i, j) - b(i, j));
      const double tol = atol + rtol * std::abs(b(i, j));
      if (diff > tol) return false;
    }
  }
  return true;
}

}  // namespace

TEST_CASE("forward kinematics at the home position is deterministic",
          "[kinematics]") {
  SixAxisArm arm;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
  Eigen::Matrix4d pose_a = arm.end_effector_pose(q);
  Eigen::Matrix4d pose_b = arm.end_effector_pose(q);
  REQUIRE(all_close(pose_a, pose_b));
}

TEST_CASE("forward kinematics output is a valid homogeneous transform",
          "[kinematics]") {
  SixAxisArm arm;
  Eigen::VectorXd q(6);
  q << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  Eigen::Matrix4d T = arm.end_effector_pose(q);

  // Bottom row must be [0, 0, 0, 1].
  Eigen::RowVector4d bottom_row = T.row(3);
  Eigen::RowVector4d expected_bottom(0.0, 0.0, 0.0, 1.0);
  REQUIRE(all_close(bottom_row, expected_bottom));

  // Rotation part must be orthonormal (R^T R = I).
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  REQUIRE(all_close(R.transpose() * R, Eigen::Matrix3d::Identity(), 1e-8));
}

TEST_CASE("inverse kinematics converges for a reachable target",
          "[kinematics]") {
  SixAxisArm arm;
  Eigen::Matrix4d target = pose_from_xyz(0.5, 0.2, 0.4);
  auto result = arm.inverse_kinematics(target);
  INFO("iterations = " << result.iterations);
  REQUIRE(result.converged);

  Eigen::Matrix4d achieved = arm.end_effector_pose(result.joint_angles);
  REQUIRE(all_close(achieved.block<3, 1>(0, 3), target.block<3, 1>(0, 3), 1e-3));
}

TEST_CASE("inverse kinematics round-trip matches forward kinematics",
          "[kinematics]") {
  // Pick a fixed joint configuration, compute its pose via FK, then solve
  // IK for that same pose and confirm the resulting pose matches (note:
  // the joint angles themselves need not match, since redundant/alternate
  // solutions can reach the same pose -- what must match is the pose).
  //
  // The Python test draws q_true from a seeded RNG (seed=42); rather than
  // reproduce NumPy's PRNG bit-for-bit in C++ (which numpy's
  // default_rng/PCG64 does not make easy to replicate exactly), this uses
  // a fixed representative configuration in the same [-1, 1] rad range,
  // which exercises the same FK->IK round-trip property.
  SixAxisArm arm;
  Eigen::VectorXd q_true(6);
  q_true << 0.37, -0.82, 0.61, -0.14, 0.95, -0.46;

  Eigen::Matrix4d target_pose = arm.end_effector_pose(q_true);
  auto result = arm.inverse_kinematics(target_pose, q_true * 0.9);

  REQUIRE(result.converged);
  Eigen::Matrix4d achieved_pose = arm.end_effector_pose(result.joint_angles);
  REQUIRE(all_close(achieved_pose, target_pose, 1e-3));
}

TEST_CASE(
    "limit clipping does not corrupt a converged solution (regression)",
    "[kinematics]") {
  // Regression test for a real bug: the damped-least-squares loop has no
  // bound on joint angle magnitude, and forward kinematics is
  // 2*pi-periodic, so it can converge at a raw angle like 8 rad (the same
  // configuration as ~1.7 rad). clip_to_limits must wrap such angles into
  // (-pi, pi] before clamping -- a plain clamp(q, -pi, pi) would snap 8
  // rad to +pi, which is a *different* configuration, silently breaking a
  // "converged" result.
  //
  // Seeding from a deliberately large initial guess reproduces the raw
  // solver output wandering past +/-pi.
  SixAxisArm arm;
  Eigen::Matrix4d target = pose_from_xyz(0.0, 0.6, 0.5);
  Eigen::VectorXd large_initial_guess(6);
  large_initial_guess << 7.5, 6.0, -17.0, 16.0, 1.5, -12.5;

  auto result = arm.inverse_kinematics(target, large_initial_guess);
  INFO("iterations = " << result.iterations);
  REQUIRE(result.converged);

  Eigen::Matrix4d achieved = arm.end_effector_pose(result.joint_angles);
  // Extra parens around the stream expression are required here: the
  // preprocessor splits macro arguments on commas it sees at the macro's
  // own paren depth, and it has no idea that the comma inside
  // `block<3, 1>` belongs to a template argument list rather than an
  // argument separator -- wrapping in `( ... )` hides that comma behind a
  // real paren before the macro ever sees it.
  Eigen::Vector3d achieved_pos = achieved.block<3, 1>(0, 3);
  Eigen::Vector3d target_pos = target.block<3, 1>(0, 3);
  INFO("achieved = " << achieved_pos.transpose());
  INFO("target   = " << target_pos.transpose());
  REQUIRE(all_close(achieved_pos, target_pos, 1e-3));
}

TEST_CASE("geometric Jacobian has the expected shape", "[kinematics]") {
  SixAxisArm arm;
  Eigen::VectorXd q(6);
  q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  Eigen::MatrixXd J = arm.geometric_jacobian(q);
  REQUIRE(J.rows() == 6);
  REQUIRE(J.cols() == 6);
}
