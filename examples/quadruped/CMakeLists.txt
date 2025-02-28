cmake_minimum_required(VERSION 3.10)
project(quadruped LANGUAGES CXX)
set(CMAKE_PREFIX_PATH "/home/erim/raisim_ws_v1.1.7/install")
set(CMAKE_BUILD_TYPE "Release")

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} "/usr/local/include")
link_directories("/usr/local/lib")

add_executable(quadrupedRobot src/main.cpp)

target_link_libraries(quadrupedRobot PUBLIC raisim::raisim pthread)
target_include_directories(quadrupedRobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(quadrupedRobot PRIVATE rbdyn)