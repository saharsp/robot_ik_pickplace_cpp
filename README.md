# robot_ik_pickplace_cpp

A C++ (Eigen + rclcpp) port of [`robot_ik_pickplace`](../robot_ik_pickplace),
the from-scratch inverse kinematics solver and simulated pick-and-place
controller for a 6-DOF arm. Same math, same node behavior, same test
coverage — rewritten because most real control loops on hardware are C++,
not Python, and closing that gap was the Python project's own stated next
step (see its README's "What to build next").

## What's actually in here

- **`include/robot_ik_pickplace_cpp/kinematics.hpp` + `src/kinematics.cpp`**
  — port of `kinematics.py`: the DH-parameter-based forward kinematics, the
  geometric Jacobian, and the damped least-squares (Levenberg-Marquardt
  style) numerical inverse kinematics solver, including the joint-angle
  wrap-before-clip fix the Python version's regression test exists for.
  Pure C++/Eigen, no ROS2 dependency, so it's buildable, testable, and
  debuggable entirely on its own.
- **`include/robot_ik_pickplace_cpp/path_metrics.hpp` + `src/path_metrics.cpp`**
  — port of `path_metrics.py`: `path_length()` over a sequence of labeled
  waypoints.
- **`standalone/demo_no_ros.cpp`** — port of `standalone_demo/demo_no_ros.py`.
  Runs the kinematics with no ROS2 install required at all: solves IK for a
  pick pose and a place pose and renders a 3D visualization. **Start here.**
- **`test/test_kinematics.cpp`** and **`test/test_path_metrics.cpp`** —
  Catch2 ports of the two pytest suites, same cases, same reasoning behind
  each hand-picked test value.
- **`src/ik_node.cpp`** — rclcpp port of `ik_node.py`: subscribes to
  `geometry_msgs/PoseStamped` on `target_pose`, solves IK, publishes
  `sensor_msgs/JointState` on `joint_states`.
- **`src/pick_place_demo.cpp`** — rclcpp port of `pick_place_demo.py`: publishes
  a scripted pick-and-place waypoint sequence to `target_pose`.
- **`urdf/six_axis_arm.urdf`** — unchanged copy of the original URDF (the
  arm geometry didn't change, only the language solving its kinematics).
- **`launch/ik_demo.launch.py`** — the same launch file, pointed at this
  package's C++ executables.
- **`CMakeLists.txt`** / **`package.xml`** — `ament_cmake` package
  definition. Notably, the `CMakeLists.txt` builds the core library,
  `demo_no_ros`, and the Catch2 tests **even outside a colcon workspace**
  (it only adds `ik_node`/`pick_place_demo` when `ament_cmake` is found) —
  see "What was and wasn't verified" below for why that matters.

## Quick start (no ROS2 needed)

This proves the math works before you touch any ROS2 setup — same role as
the Python original's "Quick start":

```bash
sudo apt install libeigen3-dev catch2   # Eigen3 + Catch2 v3
cmake -S . -B build
cmake --build build -j
./build/demo_no_ros
```

This prints the IK solutions for a pick and a place target, matching the
Python version's console output line for line, and writes
`ik_demo_result.dat` / `ik_demo_plot.gnuplot`. If `gnuplot` is installed
(`sudo apt install gnuplot`) it also renders `ik_demo_result.png` — C++ has
no built-in equivalent of matplotlib, so this is a 3D-chain plot built from
the same joint positions via gnuplot instead, rather than the exact same
plotting call.

Run the unit tests the same way:

```bash
ctest --test-dir build --output-on-failure
# or directly:
./build/unit_tests
```

## Full ROS2 demo

Requires a ROS2 install (Humble/Jazzy or current LTS) with `rclcpp`,
`robot_state_publisher`, and `rviz2`.

```bash
# from your colcon workspace's src/ directory
colcon build --packages-select robot_ik_pickplace_cpp
source install/setup.bash

# terminal 1
ros2 launch robot_ik_pickplace_cpp ik_demo.launch.py

# terminal 2
ros2 run robot_ik_pickplace_cpp pick_place_demo
```

In RViz2, add a `RobotModel` display (topic: `robot_description`) and a
`TF` display to watch the arm move through the pick-and-place sequence —
identical to the Python version's demo, just driven by the C++ node.


**One intentional divergence, noted in the test file itself:**
`test_inverse_kinematics_roundtrip_matches_forward_kinematics` in the
Python suite draws its joint configuration from `np.random.default_rng(seed=42)`.
Reproducing NumPy's PCG64 bit-for-bit in C++ isn't practical, so the C++
test uses a fixed representative configuration in the same `[-1, 1]` rad
range instead — it exercises the identical FK→IK round-trip property, just
not from the identical pseudo-random draw.

## Differences from the Python version worth knowing about

- **Linear algebra:** NumPy arrays → `Eigen::MatrixXd`/`Eigen::Matrix4d`/`Eigen::VectorXd`.
  `np.linalg.solve` → `.colPivHouseholderQr().solve(...)` for the damped
  least-squares step.
- **Angle wrapping:** `np.mod(x, y)` always returns a result with the sign
  of `y`; C++'s `std::fmod` takes the sign of `x`. `clip_to_limits`'s
  helper (`python_style_mod` in `kinematics.cpp`) replicates NumPy's
  convention explicitly rather than assuming `fmod` behaves the same way —
  this is exactly the kind of silent mismatch that would have reintroduced
  the limit-clipping bug the regression test exists to catch.
- **Optional initial guess:** Python's `initial_guess=None` default becomes
  `std::optional<Eigen::VectorXd>` in C++.
- **Visualization:** see "Quick start" above — gnuplot instead of matplotlib.

## What to build next

Same list as the Python README, now that the C++ side exists too:
swap in a real robot's DH parameters/URDF, add Gazebo physics, compare
against MoveIt2, and (optionally) an analytical IK solver for this specific
geometry as a faster alternative to the numerical one.
