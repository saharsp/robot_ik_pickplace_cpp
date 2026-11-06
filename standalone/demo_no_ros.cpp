// demo_no_ros.cpp
//
// Runs and visualizes the from-scratch kinematics WITHOUT ROS2 installed --
// C++ port of standalone_demo/demo_no_ros.py. Use this to develop/debug the
// math first -- get this working before you touch the ROS2 wiring at all.
//
// Unlike the Python original (which plots directly with matplotlib), this
// writes the plotted points to a CSV and then shells out to `gnuplot` to
// render the same kind of 3D figure, since C++ has no equivalent of
// matplotlib in the standard toolchain. If gnuplot isn't installed, the
// CSV is still written and can be plotted with any tool.
//
// Build & run (see ../CMakeLists.txt):
//     cmake --build build --target demo_no_ros
//     ./build/demo_no_ros
//
// Requires: Eigen3. Optional: gnuplot (for the PNG).

#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "robot_ik_pickplace_cpp/kinematics.hpp"

using robot_ik_pickplace_cpp::IKResult;
using robot_ik_pickplace_cpp::pose_from_xyz;
using robot_ik_pickplace_cpp::SixAxisArm;

namespace {

void print_vector(const std::string& label, const Eigen::VectorXd& v,
                   int precision = 3) {
  std::cout << label << " [";
  for (int i = 0; i < v.size(); ++i) {
    std::cout << std::fixed;
    std::cout.precision(precision);
    std::cout << v[i];
    if (i + 1 < v.size()) std::cout << ", ";
  }
  std::cout << "]\n";
}

// Writes one arm configuration's joint-chain points as a gnuplot "index"
// block (points separated by two blank lines between blocks, matching
// gnuplot's `index` data-file convention).
void write_chain_block(std::ofstream& out, const Eigen::MatrixXd& positions) {
  for (int i = 0; i < positions.rows(); ++i) {
    out << positions(i, 0) << " " << positions(i, 1) << " " << positions(i, 2)
        << "\n";
  }
  out << "\n\n";
}

}  // namespace

int main() {
  SixAxisArm arm;

  // 1. Forward kinematics sanity check: a simple pose, print the resulting
  //    end-effector transform.
  Eigen::VectorXd home_q = Eigen::VectorXd::Zero(6);
  Eigen::Matrix4d home_pose = arm.end_effector_pose(home_q);
  print_vector("Home joint angles:", home_q);
  std::cout << "End-effector pose at home:\n" << home_pose << "\n";

  // 2. Inverse kinematics: ask the arm to reach a target point (a "pick"
  //    location), starting from the home configuration.
  Eigen::Matrix4d pick_target = pose_from_xyz(0.5, 0.3, 0.4);
  Eigen::Matrix4d place_target = pose_from_xyz(-0.4, 0.4, 0.3);

  std::cout << "\nSolving IK for pick target...\n";
  IKResult pick_result = arm.inverse_kinematics(pick_target, home_q);
  std::cout << "  converged=" << (pick_result.converged ? "true" : "false")
            << " in " << pick_result.iterations << " iterations\n";
  print_vector("  joint angles (rad):", pick_result.joint_angles);
  Eigen::Matrix4d achieved = arm.end_effector_pose(pick_result.joint_angles);
  std::cout << "  achieved position: " << achieved.block<3, 1>(0, 3).transpose()
            << "\n";
  std::cout << "  target position:   " << pick_target.block<3, 1>(0, 3).transpose()
            << "\n";

  std::cout << "\nSolving IK for place target (seeded from pick solution)...\n";
  IKResult place_result =
      arm.inverse_kinematics(place_target, pick_result.joint_angles);
  std::cout << "  converged=" << (place_result.converged ? "true" : "false")
            << " in " << place_result.iterations << " iterations\n";
  print_vector("  joint angles (rad):", place_result.joint_angles);

  // 3. Write the home/pick/place joint chains and the two targets to a CSV
  //    gnuplot can consume directly (blank-line-separated blocks == gnuplot
  //    "index" ranges), then render it to a PNG if gnuplot is available.
  const std::string data_path = "ik_demo_result.dat";
  {
    std::ofstream data(data_path);
    write_chain_block(data, arm.joint_positions(home_q));
    write_chain_block(data, arm.joint_positions(pick_result.joint_angles));
    write_chain_block(data, arm.joint_positions(place_result.joint_angles));
  }

  const std::string script_path = "ik_demo_plot.gnuplot";
  {
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 900,800\n";
    script << "set output 'ik_demo_result.png'\n";
    script << "set xlabel 'X (m)'\n";
    script << "set ylabel 'Y (m)'\n";
    script << "set zlabel 'Z (m)'\n";
    script << "set title 'From-scratch 6-DOF IK: pick-and-place configurations'\n";
    script << "set view equal xyz\n";
    script << "splot \\\n";
    script << "  '" << data_path << "' index 0 with linespoints lw 2 pt 7 "
              "lc rgb 'gray' title 'Home', \\\n";
    script << "  '" << data_path << "' index 1 with linespoints lw 2 pt 7 "
              "lc rgb 'blue' title 'Pick pose (IK solution)', \\\n";
    script << "  '" << data_path << "' index 2 with linespoints lw 2 pt 7 "
              "lc rgb 'orange' title 'Place pose (IK solution)', \\\n";
    script << "  '-' with points pt 9 ps 2 lc rgb 'blue' title 'Pick target', \\\n";
    script << "  '-' with points pt 9 ps 2 lc rgb 'orange' title 'Place target'\n";
    script << pick_target(0, 3) << " " << pick_target(1, 3) << " "
           << pick_target(2, 3) << "\n";
    script << "e\n";
    script << place_target(0, 3) << " " << place_target(1, 3) << " "
           << place_target(2, 3) << "\n";
    script << "e\n";
  }

  std::cout << "\nWrote " << data_path << " and " << script_path << "\n";

  const int status = std::system(("gnuplot " + script_path + " 2>/dev/null").c_str());
  if (status == 0) {
    std::cout << "Saved visualization to ik_demo_result.png\n";
  } else {
    std::cout << "gnuplot not available or failed -- plot the two files "
                 "above with any tool to reproduce the visualization.\n";
  }

  return 0;
}
