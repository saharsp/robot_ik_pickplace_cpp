cmake_minimum_required(VERSION 3.16)
project(robot_ik_pickplace_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ---------------------------------------------------------------------------
# Core math library -- no ROS2 dependency, mirrors kinematics.py/path_metrics.py.
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core
  src/kinematics.cpp
  src/path_metrics.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen)

# ---------------------------------------------------------------------------
# Standalone (no-ROS) demo -- mirrors standalone_demo/demo_no_ros.py.
# Builds and runs with nothing but Eigen3; shells out to gnuplot (if present)
# to render ik_demo_result.png, since C++ has no matplotlib equivalent.
# ---------------------------------------------------------------------------
add_executable(demo_no_ros standalone/demo_no_ros.cpp)
target_link_libraries(demo_no_ros PRIVATE ${PROJECT_NAME}_core)

# ---------------------------------------------------------------------------
# Unit tests (Catch2) -- mirrors test/test_kinematics.py and
# test/test_path_metrics.py. Built whenever Catch2 is available, independent
# of whether this is a plain cmake build or a colcon/ament one.
# ---------------------------------------------------------------------------
option(BUILD_TESTING_LOCAL "Build the Catch2 unit tests if Catch2 is found" ON)
if(BUILD_TESTING_LOCAL)
  find_package(Catch2 3 QUIET)
  if(Catch2_FOUND)
    include(CTest)
    include(Catch)
    add_executable(unit_tests
      test/test_kinematics.cpp
      test/test_path_metrics.cpp
    )
    target_link_libraries(unit_tests PRIVATE ${PROJECT_NAME}_core Catch2::Catch2WithMain)
    catch_discover_tests(unit_tests)
  else()
    message(STATUS "${PROJECT_NAME}: Catch2 (>=3) not found -- skipping unit_tests target. "
                    "Install it with: apt install libeigen3-dev catch2")
  endif()
endif()

# ---------------------------------------------------------------------------
# ROS2 nodes -- only built inside a colcon/ament workspace, since they need
# rclcpp and friends. Everything above builds and runs fine without ROS2.
# ---------------------------------------------------------------------------
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  # Some ROS2 installs don't define the ament_target_dependencies() macro
  # (it's being phased out in favor of plain target_link_libraries with
  # each package's exported CMake targets). Use it when available; fall
  # back to the modern per-package target style otherwise, so this builds
  # either way.
  function(_robot_ik_pickplace_cpp_link_ros_deps target)
    foreach(dep ${ARGN})
      if(TARGET ${dep}::${dep})
        target_link_libraries(${target} PUBLIC ${dep}::${dep})
      elseif(DEFINED ${dep}_TARGETS)
        target_link_libraries(${target} PUBLIC ${${dep}_TARGETS})
      elseif(COMMAND ament_target_dependencies)
        ament_target_dependencies(${target} ${dep})
      else()
        target_include_directories(${target} PUBLIC ${${dep}_INCLUDE_DIRS})
        target_link_libraries(${target} PUBLIC ${${dep}_LIBRARIES})
      endif()
    endforeach()
  endfunction()

  add_executable(ik_node src/ik_node.cpp)
  target_link_libraries(ik_node PRIVATE ${PROJECT_NAME}_core)
  _robot_ik_pickplace_cpp_link_ros_deps(ik_node rclcpp sensor_msgs geometry_msgs visualization_msgs)

  add_executable(pick_place_demo src/pick_place_demo.cpp)
  target_link_libraries(pick_place_demo PRIVATE ${PROJECT_NAME}_core)
  _robot_ik_pickplace_cpp_link_ros_deps(pick_place_demo rclcpp geometry_msgs)

  install(TARGETS ik_node pick_place_demo demo_no_ros
    DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch urdf
    DESTINATION share/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_lint_auto QUIET)
    if(ament_lint_auto_FOUND)
      ament_lint_auto_find_test_dependencies()
    endif()
  endif()

  ament_package()
else()
  message(STATUS "${PROJECT_NAME}: ament_cmake not found -- building the core "
                  "library, demo_no_ros, and unit_tests only (no ik_node / "
                  "pick_place_demo). This is expected outside a colcon "
                  "workspace; see README.md.")
endif()
