cmake_minimum_required(VERSION 3.8)
project(mirena_state)

# Suppress FLANN/PCL CMake policy warning if CMake >= 3.27
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# Set C++17 standard for ROS 2 Jazzy and modern PCL compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wdeprecated-declarations)
endif()

# Global Eigen memory alignment fix to prevent PCL ABI crashes on ARM64
add_compile_definitions(EIGEN_MAX_ALIGN_BYTES=0)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mirena_common REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP REQUIRED)

# PCL compiler definitions
add_definitions(${PCL_DEFINITIONS})

# Source definitions
set(SOURCES
  src/mirena_state.cpp
  src/ekf.cpp
)

set(FAST_LIO_SOURCES
  src/fast_lio.cpp
  src/ekf.cpp
)


# Executable declarations
add_executable(mirena_state ${SOURCES})
# add_executable(mirena_state_fast_lio ${FAST_LIO_SOURCES})

# Target include directories
target_include_directories(mirena_state PRIVATE
  "src/"
)

# target_include_directories(mirena_state_fast_lio PRIVATE
#   "src/"
#   ${PCL_INCLUDE_DIRS}
# )

# ROS 2 dependency mapping (uses plain target_link_libraries internally)
ament_target_dependencies(mirena_state
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  mirena_common
)

# ament_target_dependencies(mirena_state_fast_lio
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   geometry_msgs
#   tf2_ros
#   pcl_conversions
#   mirena_common
# )

# External library linking (uses plain signature to match ament_target_dependencies)
target_link_libraries(mirena_state
  Eigen3::Eigen
)

# target_link_libraries(mirena_state_fast_lio
#   ${PCL_LIBRARIES}
#   Eigen3::Eigen
#   OpenMP::OpenMP_CXX
# )

# Install directives
install(TARGETS mirena_state DESTINATION lib/${PROJECT_NAME})

ament_package()