cmake_minimum_required(VERSION 3.16)
project(mirena_control CXX C)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Off by default: -march=native bakes the build machine's instruction set into
# the solver, and a binary built on a workstation then dies with SIGILL inside
# acados on a car whose compute unit is older. Turn it on only when the build
# happens on the machine that will run it.
option(MIRENA_CONTROL_NATIVE_ARCH
  "Tune the controller and the generated solver for the build machine" OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mirena_common REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# acados ships as a plain CMake-less tree; locate it the same way its own
# Python/Matlab interfaces do.
if(DEFINED ENV{ACADOS_SOURCE_DIR})
  set(ACADOS_ROOT "$ENV{ACADOS_SOURCE_DIR}" CACHE PATH "acados installation root")
else()
  set(ACADOS_ROOT "$ENV{HOME}/acados" CACHE PATH "acados installation root")
endif()

if(NOT EXISTS "${ACADOS_ROOT}/include/acados_c/ocp_nlp_interface.h")
  message(FATAL_ERROR
    "acados headers not found under ${ACADOS_ROOT}. "
    "Set ACADOS_SOURCE_DIR or pass -DACADOS_ROOT=/path/to/acados.")
endif()

find_library(ACADOS_LIB acados PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(HPIPM_LIB hpipm PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(BLASFEO_LIB blasfeo PATHS "${ACADOS_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

set(ACADOS_INCLUDE_DIRS
  "${ACADOS_ROOT}/include"
  "${ACADOS_ROOT}/include/acados"
  "${ACADOS_ROOT}/include/blasfeo/include"
  "${ACADOS_ROOT}/include/hpipm/include"
)

# ---------------------------------------------------------------------------
# acados code generation
#
# The vehicle model and the OCP live in src/mpc/codegen/*.py, but only as a
# source language: acados turns them into C that is compiled into the
# executable below, so the running node never loads Python, CasADi or ctypes.
# Generation reruns whenever those scripts change.
# ---------------------------------------------------------------------------
set(SOLVER_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/acados_generated")
set(SOLVER_GEN_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/src/mpc/codegen/generate_solver.py")
set(SOLVER_MODEL_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/src/mpc/codegen/dbm_model.py")

file(SHA256 "${SOLVER_GEN_SCRIPT}" _gen_hash)
file(SHA256 "${SOLVER_MODEL_SCRIPT}" _model_hash)
set(SOLVER_GEN_STAMP_VALUE "${_gen_hash}-${_model_hash}")
set(SOLVER_GEN_STAMP "${SOLVER_GEN_DIR}/.codegen.stamp")

set(_regenerate TRUE)
if(EXISTS "${SOLVER_GEN_STAMP}" AND EXISTS "${SOLVER_GEN_DIR}/acados_solver_mpc_problem.h")
  file(READ "${SOLVER_GEN_STAMP}" _previous_stamp)
  if(_previous_stamp STREQUAL SOLVER_GEN_STAMP_VALUE)
    set(_regenerate FALSE)
  endif()
endif()

if(_regenerate)
  message(STATUS "mirena_control: generating the acados solver (this takes a moment)")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env "ACADOS_SOURCE_DIR=${ACADOS_ROOT}"
            ${Python3_EXECUTABLE} "${SOLVER_GEN_SCRIPT}" --output "${SOLVER_GEN_DIR}"
    RESULT_VARIABLE _gen_result
    OUTPUT_VARIABLE _gen_output
    ERROR_VARIABLE _gen_error
  )
  if(NOT _gen_result EQUAL 0)
    message(FATAL_ERROR
      "acados code generation failed (exit ${_gen_result}).\n"
      "Requires python3 with acados_template and casadi.\n"
      "${_gen_output}\n${_gen_error}")
  endif()
  file(WRITE "${SOLVER_GEN_STAMP}" "${SOLVER_GEN_STAMP_VALUE}")
endif()

# Reconfigure if the generator changes.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${SOLVER_GEN_SCRIPT}" "${SOLVER_MODEL_SCRIPT}")

file(GLOB_RECURSE SOLVER_GEN_SOURCES "${SOLVER_GEN_DIR}/*.c")

# Remove the standalone test executables Acados generates, 
# otherwise their main() functions will conflict with your ROS node's main()
list(REMOVE_ITEM SOLVER_GEN_SOURCES
  "${SOLVER_GEN_DIR}/main_mpc_problem.c"
  "${SOLVER_GEN_DIR}/main_sim_mpc_problem.c"
)

add_library(mpc_solver_generated STATIC ${SOLVER_GEN_SOURCES})
target_include_directories(mpc_solver_generated PUBLIC
  "${SOLVER_GEN_DIR}" ${ACADOS_INCLUDE_DIRS})
target_link_libraries(mpc_solver_generated PUBLIC
  ${ACADOS_LIB} ${HPIPM_LIB} ${BLASFEO_LIB} m)
# CasADi's output is machine written; its warnings are not actionable.
target_compile_options(mpc_solver_generated PRIVATE -O3 -w)

# ---------------------------------------------------------------------------
# Shared code
#
# src/common holds what more than one controller needs: the centreline spline
# and the seqlock both the MPC and the PID node hand their plans across on.
# Everything specific to one controller lives beside that controller's node.
# ---------------------------------------------------------------------------
add_library(mirena_control_common STATIC
  src/common/cubic_spline.cpp
  src/common/pid.cpp
)
target_include_directories(mirena_control_common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_compile_options(mirena_control_common PRIVATE -O3 -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# MPC controller library
#
# Headers sit next to their sources in src/mpc: nothing outside this package
# includes them, so they are private to the MPC executable rather than an
# installed interface.
# ---------------------------------------------------------------------------
add_library(mirena_control_core STATIC
  src/mpc/mpc_controller.cpp
)
target_include_directories(mirena_control_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpc>")
target_link_libraries(mirena_control_core PUBLIC mirena_control_common mpc_solver_generated)
target_compile_options(mirena_control_core PRIVATE -O3 -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# PID controller library
#
# Deliberately free of acados: the PID exists to be the controller that still
# builds and still runs when the solver is the thing under suspicion, so it must
# not acquire a dependency on it.
# ---------------------------------------------------------------------------
add_library(mirena_control_pid_core STATIC
  src/pid/pid_controller.cpp
)
target_include_directories(mirena_control_pid_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/pid>")
target_link_libraries(mirena_control_pid_core PUBLIC mirena_control_common)
target_compile_options(mirena_control_pid_core PRIVATE -O3 -Wall -Wextra -Wpedantic)

if(MIRENA_CONTROL_NATIVE_ARCH)
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-march=native HAVE_MARCH_NATIVE)
  if(HAVE_MARCH_NATIVE)
    target_compile_options(mpc_solver_generated PRIVATE -march=native)
    target_compile_options(mirena_control_common PRIVATE -march=native)
    target_compile_options(mirena_control_core PRIVATE -march=native)
    target_compile_options(mirena_control_pid_core PRIVATE -march=native)
  endif()
endif()

# ---------------------------------------------------------------------------
# Executables
#
# Three alternative occupants of the same slot in the stack: exactly one runs
# per mission, all three under the node name `mirena_control` that the
# supervisor monitors -- which is why none of them is named that.
# mirena_control_mpc drives the track under the solver; mirena_control_pid
# drives the same track with PID loops instead, on the same topics and the same
# mission parameters; mirena_control_inspection sweeps the steering on stands.
# All three report mission_finished in their heartbeat, which is all the
# supervisor reads.
# ---------------------------------------------------------------------------
add_executable(mirena_control_mpc src/mpc/mpc_node.cpp)
target_link_libraries(mirena_control_mpc mirena_control_core nlohmann_json::nlohmann_json)
target_compile_options(mirena_control_mpc PRIVATE -O3 -Wall -Wextra -Wpedantic)
ament_target_dependencies(mirena_control_mpc
  rclcpp geometry_msgs visualization_msgs tf2 tf2_ros mirena_common)

add_executable(mirena_control_pid src/pid/pid_node.cpp)
target_link_libraries(mirena_control_pid mirena_control_pid_core nlohmann_json::nlohmann_json)
target_compile_options(mirena_control_pid PRIVATE -O3 -Wall -Wextra -Wpedantic)
ament_target_dependencies(mirena_control_pid
  rclcpp geometry_msgs visualization_msgs tf2 tf2_ros mirena_common)

# No solver, no track, no TF: the inspection node needs none of those. It does
# need the scalar PID out of src/common, which it closes on the CAN bridge's
# wheel speeds.
add_executable(mirena_control_inspection src/inspection/inspection_node.cpp)
target_link_libraries(mirena_control_inspection
  mirena_control_common nlohmann_json::nlohmann_json)
target_compile_options(mirena_control_inspection PRIVATE -O2 -Wall -Wextra -Wpedantic)
ament_target_dependencies(mirena_control_inspection
  rclcpp mirena_common)

install(TARGETS mirena_control_mpc mirena_control_pid mirena_control_inspection
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
