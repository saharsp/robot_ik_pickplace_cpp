// test_path_metrics.cpp
//
// Catch2 port of test/test_path_metrics.py -- same six cases, same reasoning
// behind each chosen value (see the project README for the derivations).

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "robot_ik_pickplace_cpp/path_metrics.hpp"

using robot_ik_pickplace_cpp::path_length;
using robot_ik_pickplace_cpp::Waypoint;
using Catch::Approx;

TEST_CASE("path_length of an empty list is zero", "[path_metrics]") {
  REQUIRE(path_length({}) == 0.0);
}

TEST_CASE("path_length of a single waypoint is zero", "[path_metrics]") {
  std::vector<Waypoint> waypoints = {{"home", 0.9, -0.2, 0.05}};
  REQUIRE(path_length(waypoints) == 0.0);
}

TEST_CASE("path_length of two waypoints matches Euclidean distance",
          "[path_metrics]") {
  std::vector<Waypoint> waypoints = {
      {"a", 0.0, 0.0, 0.0},
      {"b", 3.0, 4.0, 0.0},
  };
  // 3-4-5 triangle -> exactly 5.0
  REQUIRE(path_length(waypoints) == Approx(5.0));
}

TEST_CASE(
    "path_length sums consecutive segments, not endpoint to endpoint",
    "[path_metrics]") {
  // Regression-style check: the total must be the sum of each leg, not a
  // shortcut straight line from the first to the last waypoint. Three
  // points on a right-angle dogleg make the difference obvious: hop
  // A->B->C (dogleg) sums to 2.0, while the direct A->C distance is only
  // sqrt(2).
  std::vector<Waypoint> waypoints = {
      {"a", 0.0, 0.0, 0.0},
      {"b", 1.0, 0.0, 0.0},
      {"c", 1.0, 1.0, 0.0},
  };
  REQUIRE(path_length(waypoints) == Approx(2.0));
  REQUIRE(path_length(waypoints) != Approx(std::sqrt(2.0)));
}

TEST_CASE("path_length matches the real pick_place_demo waypoints",
          "[path_metrics]") {
  // Sanity-check against the real waypoint list used in
  // pick_place_demo.cpp, computed independently here via a plain
  // leg-by-leg sum rather than by calling path_length on a subset of
  // itself.
  std::vector<Waypoint> waypoints = {
      {"approach pick", 0.5, 0.3, 0.55},
      {"pick", 0.5, 0.3, 0.40},
      {"lift", 0.5, 0.3, 0.55},
      {"approach place", -0.4, 0.4, 0.45},
      {"place", -0.4, 0.4, 0.30},
      {"retreat", -0.4, 0.4, 0.45},
      {"home", 0.9, -0.2, 0.05},
  };

  double expected = 0.0;
  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const double dx = waypoints[i + 1].x - waypoints[i].x;
    const double dy = waypoints[i + 1].y - waypoints[i].y;
    const double dz = waypoints[i + 1].z - waypoints[i].z;
    expected += std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  REQUIRE(path_length(waypoints) == Approx(expected));
}

TEST_CASE("path_length is order dependent", "[path_metrics]") {
  // Reversing the order changes nothing here (distance is symmetric per
  // leg), but shuffling into a different visiting order should generally
  // change the total -- guards against an implementation that
  // accidentally sorts points or computes some order-independent quantity
  // instead.
  std::vector<Waypoint> ordered = {
      {"a", 0.0, 0.0, 0.0},
      {"b", 1.0, 0.0, 0.0},
      {"c", 1.0, 1.0, 0.0},
  };
  std::vector<Waypoint> shuffled = {ordered[1], ordered[0], ordered[2]};

  REQUIRE(path_length(ordered) != Approx(path_length(shuffled)));
}
