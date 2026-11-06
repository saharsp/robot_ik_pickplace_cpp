// path_metrics.hpp
//
// Small, pure C++/Eigen utilities for reasoning about a planned sequence of
// Cartesian waypoints -- no ROS2 dependency, same philosophy as
// kinematics.hpp: keep anything you'd want to unit test or sanity-check
// offline (e.g. "how far is the end-effector actually going to travel for
// this pick-and-place cycle?") separate from the ROS2 node wiring.
//
// C++ port of robot_ik_pickplace/path_metrics.py.

#pragma once

#include <string>
#include <vector>

namespace robot_ik_pickplace_cpp {

// One labeled Cartesian waypoint, mirroring the Python (label, x, y, z)
// tuples used in pick_place_demo.py.
struct Waypoint {
  std::string label;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// Given a list of (label, x, y, z) waypoints, like kPickPlaceWaypoints in
// pick_place_demo.cpp, return the total Euclidean distance traveled by the
// end-effector moving through them in order.
double path_length(const std::vector<Waypoint>& waypoints);

}  // namespace robot_ik_pickplace_cpp
