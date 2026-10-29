cmake_minimum_required(VERSION 3.8)
project(mirena_slam)

# GTSAM and Eigen are template-heavy: an unoptimised build is several times
# slower, which is the difference between hitting the keyframe budget and not.
# Default to Release rather than inheriting CMake's empty build type.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wdeprecated-declarations)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(mirena_common REQUIRED)

# The estimator, with no ROS in it. Split out from the node so it can be linked
# into a test binary and driven directly -- everything worth asserting about
# this package is a property of the estimator, not of the wiring.
add_library(mirena_slam_core
  src/slam.cpp
  src/association.cpp
)
target_include_directories(mirena_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_include_directories(mirena_slam_core SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)
# GTSAM handles its own export targets best this way.
target_link_libraries(mirena_slam_core gtsam)

add_executable(mirena_slam src/mirena_slam.cpp)
target_link_libraries(mirena_slam mirena_slam_core)

ament_target_dependencies(mirena_slam
  rclcpp
  visualization_msgs
  geometry_msgs
  mirena_common
  message_filters
  tf2_ros
  nav_msgs
)

install(TARGETS mirena_slam DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mirena_slam
    test/test_association.cpp
    test/test_candidates.cpp
    test/test_scenarios.cpp
    test/test_isam.cpp
  )
  target_link_libraries(test_mirena_slam mirena_slam_core)
  # Track layouts exported from the sim, shared with mirena_planning's tests.
  # Read as data at run time rather than depended on as a package, so the two
  # test suites cannot drift into disagreeing about what a track is.
  target_compile_definitions(test_mirena_slam PRIVATE
    MIRENA_SLAM_TRACK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../mirena_planning/test/tracks")
endif()

# Register package layout
ament_package()
