cmake_minimum_required(VERSION 3.10)
project(quadruped LANGUAGES CXX)

add_executable(quadrupedRobot src/main.cpp)

target_link_libraries(quadrupedRobot PUBLIC raisim::raisim pthread)
target_include_directories(quadrupedRobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(quadrupedRobot PRIVATE rbdyn)