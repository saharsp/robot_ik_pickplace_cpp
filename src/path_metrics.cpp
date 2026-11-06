#include "robot_ik_pickplace_cpp/path_metrics.hpp"

#include <cmath>

namespace robot_ik_pickplace_cpp {

double path_length(const std::vector<Waypoint>& waypoints) {
  if (waypoints.size() < 2) {
    return 0.0;
  }

  double total = 0.0;
  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const Waypoint& a = waypoints[i];
    const Waypoint& b = waypoints[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    total += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return total;
}

}  // namespace robot_ik_pickplace_cpp
