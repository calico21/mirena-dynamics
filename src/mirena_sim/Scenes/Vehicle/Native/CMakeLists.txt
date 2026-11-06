cmake_minimum_required(VERSION 3.22)
project(ter26_vehicle_dynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)

set(GODOT_CPP_DIR "/home/alexr/eRacing/TeR_TWIN/cpp/external/godot-cpp")
set(GODOT_CPP_BUILD_DIR "/home/alexr/eRacing/TeR_TWIN/cpp/build/godot-cpp")

# 1. Pure C++20 Physics Core Library (100% Godot-independent)
add_library(ter26_core STATIC
    src/core/suspension_kinematics.cpp
    src/core/hpitm_tire.cpp
    src/core/component_multibody.cpp
    src/core/contact_solver.cpp
    src/core/vehicle_dynamics_engine.cpp
)

target_include_directories(ter26_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/generated/symforce
    ${EIGEN3_INCLUDE_DIR}
)

target_compile_options(ter26_core PRIVATE -O3 -Wall -Wextra -Wno-unused-parameter)

# 2. Comprehensive C++20 Unit, Numerical, Physical, Tire & 400 Hz Benchmark Test Suite
add_executable(test_ter26_suite
    tests/test_ter26_suite.cpp
)
target_link_libraries(test_ter26_suite PRIVATE ter26_core)
target_compile_options(test_ter26_suite PRIVATE -O3 -Wall -Wextra)

# 2b. Real Telemetry Replay Driver for Experimental Correlation
add_executable(run_telemetry_replay
    tests/run_telemetry_replay.cpp
)
target_link_libraries(run_telemetry_replay PRIVATE ter26_core)
target_compile_options(run_telemetry_replay PRIVATE -O3 -Wall -Wextra)

# 3. Godot 4.7 GDExtension Shared Library (libter26_vehicle.so)
if(EXISTS "${GODOT_CPP_BUILD_DIR}/bin/libgodot-cpp.linux.debug.64.a")
    add_library(ter26_vehicle SHARED
        src/godot/vehicle_model_binding.cpp
    )
    target_include_directories(ter26_vehicle PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/generated/symforce
        ${CMAKE_CURRENT_SOURCE_DIR}/src/godot
        ${EIGEN3_INCLUDE_DIR}
        "${GODOT_CPP_DIR}/include"
        "${GODOT_CPP_DIR}/gdextension"
        "${GODOT_CPP_BUILD_DIR}/gen/include"
    )
    target_link_libraries(ter26_vehicle PRIVATE
        -Wl,--whole-archive ter26_core -Wl,--no-whole-archive
        "${GODOT_CPP_BUILD_DIR}/bin/libgodot-cpp.linux.debug.64.a"
    )
    target_compile_options(ter26_vehicle PRIVATE -O3)
    set_target_properties(ter26_vehicle PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
        OUTPUT_NAME "ter26_vehicle"
    )
endif()
