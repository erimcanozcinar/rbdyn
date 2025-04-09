cmake_minimum_required(VERSION 3.10)
project(floatingBase LANGUAGES CXX)

add_executable(floatingBaseRobot src/main.cpp)

target_link_libraries(floatingBaseRobot PUBLIC raisim::raisim pthread)
target_include_directories(floatingBaseRobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(floatingBaseRobot PRIVATE rbdyn)