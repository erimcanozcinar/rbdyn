cmake_minimum_required(VERSION 3.10)
project(fixedBase LANGUAGES CXX)

add_executable(fixedBaseRobot src/main.cpp)

target_link_libraries(fixedBaseRobot PUBLIC raisim::raisim pthread)
target_include_directories(fixedBaseRobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(fixedBaseRobot PRIVATE rbdyn)