cmake_minimum_required(VERSION 3.8)
project(mirena_hardware)

# ---------------------------------------------------------------------------
# C / C++ standards
# ---------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# ---------------------------------------------------------------------------
# Warnings (only for our own code; third-party SDK is excluded below)
# ---------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Architecture-specific optimisation
#   NOTE: -ffast-math breaks strict IEEE behaviour (NaN/Inf handling, assoc.
#   reordering). It is enabled here to match the original build, but if any
#   sensor-fusion / filtering code relies on NaN checks, drop it.
#   -march=native bakes in the *build* host's ISA; only safe when build host
#   == run host. The Jetson branch is pinned explicitly, which is correct.
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  # Jetson / ARM
  add_compile_options(-O3 -march=armv8-a+simd -ffast-math)
else()
  # x86
  add_compile_options(-O3 -march=native -ffast-math)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mirena_common REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---------------------------------------------------------------------------
# Common include dirs for OUR code (SDK headers are added per-target as SYSTEM)
# ---------------------------------------------------------------------------
include_directories(
  ${tf2_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  src
)

# ===========================================================================
# Targets
# ===========================================================================

# --- LiDAR -----------------------------------------------------------------
add_executable(mirena_lidar
  src/lidar/mirena_lidar.cpp
  src/lidar/pointcloud_builder.cpp
)
ament_target_dependencies(mirena_lidar
  rclcpp
  rclcpp_lifecycle
  mirena_common
  sensor_msgs
)

# --- Camera ----------------------------------------------------------------
add_executable(mirena_camera
  src/camera/mirena_camera.cpp
)
ament_target_dependencies(mirena_camera
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  camera_info_manager
  mirena_common
)

# --- IMU (VectorNav SDK) ---------------------------------------------------
# The SDK uses non-ISO extensions (anonymous structs, etc.). We:
#   1. add its include dir as SYSTEM so its headers don't trip OUR warnings,
#   2. compile its own .cpp files with warnings disabled (-w).
# CONFIGURE_DEPENDS makes the glob re-run when files are added/removed.
file(GLOB_RECURSE VECNAV_SOURCES CONFIGURE_DEPENDS
  "src/imu/vecnav_sdk/cpp/src/**/*.cpp")

set_source_files_properties(${VECNAV_SOURCES} PROPERTIES
  COMPILE_OPTIONS "-w")

add_executable(mirena_imu
  src/imu/mirena_imu.cpp
  ${VECNAV_SOURCES}
)
target_include_directories(mirena_imu SYSTEM PRIVATE
  src/imu/vecnav_sdk/cpp/include
)
ament_target_dependencies(mirena_imu
  rclcpp
  tf2_ros
  tf2
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  mirena_common
)

# --- Transform broadcaster -------------------------------------------------
add_executable(mirena_transform
  src/transform/transform_broadcaster.cpp
)
ament_target_dependencies(mirena_transform
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  tf2_ros
  ament_index_cpp
)
target_link_libraries(mirena_transform yaml-cpp)

# --- CAN -------------------------------------------------------------------
# ter.c is C; the DBC dir holds generated headers. The CAN link is expected to
# be already up, so the node only needs raw SocketCAN (kernel) -- no libsocketcan.
add_executable(mirena_can
  src/can/mirena_can.cpp
  src/can/can_logger.cpp
  src/can/dbc/ter.c
)
target_include_directories(mirena_can PRIVATE src/can/dbc)
ament_target_dependencies(mirena_can
  rclcpp
  mirena_common
)
# The logger writes on its own thread.
find_package(Threads REQUIRED)
target_link_libraries(mirena_can Threads::Threads)

# ===========================================================================
# Install
# ===========================================================================
install(TARGETS
  mirena_lidar
  mirena_camera
  mirena_imu
  mirena_transform
  mirena_can
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ===========================================================================
# Testing (lint hooks, opt-in)
# ===========================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cpplint/uncrustify on third-party SDK code is noise; skip copyright too.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()