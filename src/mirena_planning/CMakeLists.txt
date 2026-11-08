cmake_minimum_required(VERSION 3.8)
project(mirena_planning)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(mirena_common REQUIRED)

# ---------------------------------------------------------------------------
# Planning core library
# ---------------------------------------------------------------------------
add_library(mirena_planning_core SHARED
  src/geometry.cpp
  src/planner.cpp
  src/unknown/unknown_planner.cpp
  src/known/known_planner.cpp
  src/known/track_template.cpp
  src/known/known_planner_acceleration.cpp
)

target_include_directories(mirena_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Apply dependencies to the core library so all targets share matching build flags
ament_target_dependencies(mirena_planning_core
  mirena_common
  geometry_msgs
  std_msgs
  visualization_msgs
)

# Plain signature (no PUBLIC) to match ament_target_dependencies
target_link_libraries(mirena_planning_core nlohmann_json::nlohmann_json)

# ---------------------------------------------------------------------------
# ROS 2 nodes
# ---------------------------------------------------------------------------
set(PLANNER_NODE_DEPS
  ament_index_cpp
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  mirena_common
)

add_executable(mirena_planning src/unknown/unknown_planner_node.cpp)
target_link_libraries(mirena_planning mirena_planning_core nlohmann_json::nlohmann_json)
ament_target_dependencies(mirena_planning ${PLANNER_NODE_DEPS})

add_executable(mirena_planning_known_skidpad src/known/known_planner_node.cpp)
target_link_libraries(mirena_planning_known_skidpad mirena_planning_core nlohmann_json::nlohmann_json)
ament_target_dependencies(mirena_planning_known_skidpad ${PLANNER_NODE_DEPS})

add_executable(mirena_planning_known_acceleration src/known/known_planner_node_acceleration.cpp)
target_link_libraries(mirena_planning_known_acceleration mirena_planning_core nlohmann_json::nlohmann_json)
ament_target_dependencies(mirena_planning_known_acceleration ${PLANNER_NODE_DEPS})

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------
install(TARGETS mirena_planning mirena_planning_known_skidpad mirena_planning_known_acceleration
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mirena_planning_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mirena_planning
    test/test_geometry.cpp
    test/test_unknown_planner.cpp
    test/test_known_planner.cpp
    test/test_real_tracks.cpp
  )
  target_link_libraries(test_mirena_planning mirena_planning_core)
  target_compile_definitions(test_mirena_planning PRIVATE
    MIRENA_PLANNING_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resource"
    MIRENA_PLANNING_TRACK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/tracks"
  )
endif()

ament_package()
