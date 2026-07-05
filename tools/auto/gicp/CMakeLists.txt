# Standalone build for the offline GICP alignment shim.
# Deliberately NOT a colcon/ament package — this is an offline tool that only
# reuses the already-installed small_gicp C++ library (the same one the
# small_gicp_relocalization ROS node links). Build once:
#
#   cmake -S tools/auto/gicp -B tools/auto/gicp/build
#   cmake --build tools/auto/gicp/build
#
# Produces tools/auto/gicp/build/small_gicp_align, which auto_align_gicp.py
# calls via subprocess.

cmake_minimum_required(VERSION 3.10)
project(small_gicp_align CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(small_gicp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(small_gicp_align small_gicp_align.cpp)
target_link_libraries(small_gicp_align
  small_gicp::small_gicp
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)
