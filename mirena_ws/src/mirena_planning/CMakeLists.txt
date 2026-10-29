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
# Planning core: no ROS, so the geometry can be unit tested without a node.
# ---------------------------------------------------------------------------
add_library(mirena_planning_core
  src/geometry.cpp
  src/planner.cpp
  src/unknown/unknown_planner.cpp
  src/known/known_planner.cpp
  src/known/track_template.cpp
)
target_include_directories(mirena_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mirena_planning_core nlohmann_json::nlohmann_json)

# ---------------------------------------------------------------------------
# ROS 2 nodes
#
# Two alternative occupants of the same slot in the stack: exactly one runs per
# mission, under the node name the supervisor monitors. mirena_planning
# discovers the track (trackdrive, autocross); mirena_planning_known registers a
# surveyed template against it (skidpad, acceleration). Both publish the same
# planning/track and health topics, and neither shares a line of node code with
# the other -- each is one self-contained translation unit.
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

add_executable(mirena_planning_known src/known/known_planner_node.cpp)
target_link_libraries(mirena_planning_known mirena_planning_core nlohmann_json::nlohmann_json)
ament_target_dependencies(mirena_planning_known ${PLANNER_NODE_DEPS})

install(TARGETS mirena_planning mirena_planning_known DESTINATION lib/${PROJECT_NAME})
install(TARGETS mirena_planning_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
    # Layouts exported from the sim, with the centreline they were drawn around.
    MIRENA_PLANNING_TRACK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/tracks")
endif()

ament_package()
