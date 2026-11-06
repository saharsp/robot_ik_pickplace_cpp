#include "robot_ik_pickplace_cpp/kinematics.hpp"

#include <cassert>
#include <cmath>

namespace robot_ik_pickplace_cpp {

namespace {

// Python's np.mod(x, y) always returns a result with the same sign as y
// (i.e. in [0, y) for y > 0). std::fmod instead takes the sign of x, so a
// direct fmod(q + pi, 2*pi) - pi would NOT reproduce the Python wrapping
// for negative q. This helper matches np.mod's convention exactly.
double python_style_mod(double x, double y) {
  double r = std::fmod(x, y);
  if (r != 0.0 && ((r < 0.0) != (y < 0.0))) {
    r += y;
  }
  return r;
}

}  // namespace

std::vector<DHParam> default_dh_table() {
  const double half_pi = M_PI / 2.0;
  return {
      DHParam{0.0, 0.15, 0.0, half_pi},
      DHParam{0.0, 0.0, 0.50, 0.0},
      DHParam{0.0, 0.0, 0.40, 0.0},
      DHParam{0.0, 0.10, 0.0, half_pi},
      DHParam{0.0, 0.10, 0.0, -half_pi},
      DHParam{0.0, 0.10, 0.0, 0.0},
  };
}

JointLimits default_joint_limits() {
  return JointLimits(6, std::make_pair(-M_PI, M_PI));
}

Eigen::Matrix4d dh_transform(double theta, double d, double a, double alpha) {
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);

  Eigen::Matrix4d T;
  T << ct, -st * ca, st * sa, a * ct,
       st, ct * ca, -ct * sa, a * st,
       0.0, sa, ca, d,
       0.0, 0.0, 0.0, 1.0;
  return T;
}

SixAxisArm::SixAxisArm(std::vector<DHParam> dh_table, JointLimits joint_limits)
    : dh_table_(std::move(dh_table)),
      joint_limits_(std::move(joint_limits)),
      n_joints_(static_cast<int>(dh_table_.size())) {}

std::vector<Eigen::Matrix4d> SixAxisArm::forward_kinematics(
    const Eigen::VectorXd& joint_angles) const {
  assert(joint_angles.size() == n_joints_);

  std::vector<Eigen::Matrix4d> transforms;
  transforms.reserve(n_joints_);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < n_joints_; ++i) {
    const DHParam& dh = dh_table_[i];
    Eigen::Matrix4d A =
        dh_transform(joint_angles[i] + dh.theta_offset, dh.d, dh.a, dh.alpha);
    T = T * A;
    transforms.push_back(T);
  }
  return transforms;
}

Eigen::Matrix4d SixAxisArm::end_effector_pose(
    const Eigen::VectorXd& joint_angles) const {
  return forward_kinematics(joint_angles).back();
}

Eigen::MatrixXd SixAxisArm::joint_positions(
    const Eigen::VectorXd& joint_angles) const {
  std::vector<Eigen::Matrix4d> transforms = forward_kinematics(joint_angles);

  Eigen::MatrixXd positions(n_joints_ + 1, 3);
  positions.row(0) = Eigen::RowVector3d::Zero();
  for (int i = 0; i < n_joints_; ++i) {
    positions.row(i + 1) = transforms[i].block<3, 1>(0, 3).transpose();
  }
  return positions;
}

Eigen::MatrixXd SixAxisArm::geometric_jacobian(
    const Eigen::VectorXd& joint_angles) const {
  std::vector<Eigen::Matrix4d> transforms = forward_kinematics(joint_angles);
  const Eigen::Vector3d o_n = transforms.back().block<3, 1>(0, 3);

  std::vector<Eigen::Vector3d> z_axes;
  std::vector<Eigen::Vector3d> origins;
  z_axes.reserve(n_joints_);
  origins.reserve(n_joints_);

  z_axes.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));  // z0 in base frame
  origins.push_back(Eigen::Vector3d::Zero());        // o0 in base frame
  for (int i = 0; i < n_joints_ - 1; ++i) {
    z_axes.push_back(transforms[i].block<3, 1>(0, 2));
    origins.push_back(transforms[i].block<3, 1>(0, 3));
  }

  Eigen::MatrixXd J(6, n_joints_);
  for (int i = 0; i < n_joints_; ++i) {
    const Eigen::Vector3d& z_im1 = z_axes[i];
    const Eigen::Vector3d& o_im1 = origins[i];
    J.block<3, 1>(0, i) = z_im1.cross(o_n - o_im1);
    J.block<3, 1>(3, i) = z_im1;
  }
  return J;
}

IKResult SixAxisArm::inverse_kinematics(
    const Eigen::Matrix4d& target_pose,
    std::optional<Eigen::VectorXd> initial_guess, int max_iters, double tol,
    double damping) const {
  Eigen::VectorXd q =
      initial_guess.has_value() ? *initial_guess : Eigen::VectorXd::Zero(n_joints_);

  for (int it = 0; it < max_iters; ++it) {
    Eigen::Matrix4d current_pose = end_effector_pose(q);
    Eigen::VectorXd error = pose_error(current_pose, target_pose);

    if (error.norm() < tol) {
      return IKResult{clip_to_limits(q), true, it};
    }

    Eigen::MatrixXd J = geometric_jacobian(q);
    Eigen::MatrixXd JJt = J * J.transpose();
    Eigen::MatrixXd damped =
        JJt + (damping * damping) * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J.transpose() * damped.colPivHouseholderQr().solve(error);
    q = q + dq;
  }

  return IKResult{clip_to_limits(q), false, max_iters};
}

Eigen::VectorXd SixAxisArm::clip_to_limits(const Eigen::VectorXd& q) const {
  Eigen::VectorXd result(n_joints_);
  for (int i = 0; i < n_joints_; ++i) {
    double wrapped = python_style_mod(q[i] + M_PI, 2.0 * M_PI) - M_PI;
    const auto& [lo, hi] = joint_limits_[i];
    result[i] = std::min(std::max(wrapped, lo), hi);
  }
  return result;
}

Eigen::VectorXd SixAxisArm::pose_error(const Eigen::Matrix4d& current,
                                        const Eigen::Matrix4d& target) {
  Eigen::Vector3d pos_err =
      target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);

  Eigen::Matrix3d R_err =
      target.block<3, 3>(0, 0) * current.block<3, 3>(0, 0).transpose();
  // Small-angle axis-angle extraction from the skew-symmetric part.
  Eigen::Vector3d rot_err(R_err(2, 1) - R_err(1, 2), R_err(0, 2) - R_err(2, 0),
                           R_err(1, 0) - R_err(0, 1));
  rot_err *= 0.5;

  Eigen::VectorXd error(6);
  error << pos_err, rot_err;
  return error;
}

Eigen::Matrix4d pose_from_xyz(double x, double y, double z,
                               const Eigen::Matrix3d& R) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return T;
}

}  // namespace robot_ik_pickplace_cpp
