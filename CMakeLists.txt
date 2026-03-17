cmake_minimum_required(VERSION 3.16)
project(uwb_eskf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
target_link_libraries(data_converter PRIVATE uwb_utils Eigen3::Eigen)

add_executable(jacobian_audit apps/jacobian_audit_main.cpp)
target_link_libraries(jacobian_audit PRIVATE uwb_app)

add_executable(regression_checks apps/regression_checks_main.cpp)
target_link_libraries(regression_checks PRIVATE uwb_app)
