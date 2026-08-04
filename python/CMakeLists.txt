cmake_minimum_required(VERSION 3.16)
project(invariant_smoother_py LANGUAGES CXX)

# pybind11 module for Yoon et al. (2024) InvariantSmoother.
#
# The estimator sources are compiled EXACTLY AS SHIPPED — no include shadows, no
# patched constants (in particular WINDOW_SIZE stays at upstream's 5). Anything
# that changes their code changes the baseline, so the only addition is
# bindings.cpp, which drives the public Onestep() API and nothing else.
#
# Build (from the repo root) — build tree stays inside the submodule:
#   cmake -S third_party/invariant_smoother/python \
#         -B third_party/invariant_smoother/build \
#         -DPython_EXECUTABLE=$PWD/.venv/bin/python -DCMAKE_BUILD_TYPE=Release
#   cmake --build third_party/invariant_smoother/build -j

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(IS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(REPO_ROOT ${IS_ROOT}/../..)
if(NOT EXISTS ${IS_ROOT}/src/estimator/InvariantSmoother.cpp)
  message(FATAL_ERROR
    "invariant_smoother sources missing — run: git submodule update --init")
endif()

find_package(Eigen3 REQUIRED)
# Reuse the pybind11 already vendored for the gtsam wrapper.
add_subdirectory(${REPO_ROOT}/third_party/gtsam/wrap/pybind11
                 ${CMAKE_BINARY_DIR}/pybind11)

pybind11_add_module(invariant_smoother_py
  bindings.cpp
  ${IS_ROOT}/src/estimator/InvariantSmoother.cpp
  ${IS_ROOT}/src/utility/EstimatorCommonStruct.cpp)

target_include_directories(invariant_smoother_py PRIVATE ${IS_ROOT}/include)
target_link_libraries(invariant_smoother_py PRIVATE Eigen3::Eigen)

# Land the .so beside this CMakeLists so the Python wrapper can import it.
set_target_properties(invariant_smoother_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
