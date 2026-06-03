cmake_minimum_required(VERSION 3.16)
project(combined_ins LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# CMake 4.x rejects older policy declarations used by some vendored
# FetchContent projects unless a minimum policy compatibility version is set.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING
    "Minimum policy version accepted for fetched dependencies")

include(FetchContent)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

add_library(uwb_utils
  src/utils/math_utils.cpp
)
target_include_directories(uwb_utils PUBLIC include ${eigen_SOURCE_DIR})
target_link_libraries(uwb_utils PUBLIC Eigen3::Eigen)

add_library(uwb_core
  src/core/ins_mech.cpp
  src/core/eskf_engine.cpp
  src/core/measurement_models_uwb.cpp
  src/core/anchors.cpp
  src/navigation/filter_contracts.cpp
  src/navigation/filter_engine.cpp
  src/navigation/filter_reset.cpp
  src/navigation/measurement_model_common.cpp
  src/navigation/measurement_model_gnss.cpp
  src/navigation/measurement_model_road.cpp
  src/navigation/measurement_model_uwb.cpp
  src/navigation/nominal_propagation.cpp
  src/navigation/process_model_standard_eskf.cpp
  src/navigation/process_model_inekf.cpp
  src/navigation/process_noise_mapping.cpp
)
target_include_directories(uwb_core PUBLIC include ${eigen_SOURCE_DIR})
target_link_libraries(uwb_core PUBLIC uwb_utils Eigen3::Eigen)

add_library(uwb_io
  src/io/data_io.cpp
)
target_include_directories(uwb_io PUBLIC include ${eigen_SOURCE_DIR})
target_link_libraries(uwb_io PUBLIC Eigen3::Eigen)

add_library(uwb_app
  src/app/config.cpp
  src/app/dataset_loader.cpp
  src/app/initialization.cpp
  src/app/fusion_runtime.cpp
  src/app/fusion_scheduler.cpp
  src/app/fusion_update_runner.cpp
  src/app/fusion_phase_controls.cpp
  src/app/fusion_diagnostics_runtime.cpp
  src/app/pipeline_fusion.cpp
  src/app/evaluation.cpp
  src/app/diagnostics.cpp
)
target_include_directories(uwb_app PUBLIC include ${eigen_SOURCE_DIR})
target_link_libraries(uwb_app PUBLIC uwb_core uwb_utils uwb_io yaml-cpp)

add_executable(eskf_fusion apps/eskf_fusion_main.cpp)
target_link_libraries(eskf_fusion PRIVATE uwb_app)

add_executable(uwb_generator apps/uwb_generator_main.cpp)
target_link_libraries(uwb_generator PRIVATE uwb_app)

add_executable(data_converter apps/data_converter_main.cpp)
target_link_libraries(data_converter PRIVATE uwb_utils uwb_io Eigen3::Eigen)
