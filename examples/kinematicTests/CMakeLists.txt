cmake_minimum_required(VERSION 3.10)
project(kinematicTest LANGUAGES CXX)
set(CMAKE_PREFIX_PATH "/home/erim/raisim_ws_v1.1.7/install")
set(CMAKE_BUILD_TYPE "Release")

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} "/usr/local/include")
link_directories("/usr/local/lib")

add_executable(kinematicTest src/main.cpp)
add_executable(quadrupedKinTest src/main_quadruped.cpp)

target_link_libraries(kinematicTest PUBLIC raisim::raisim pthread)
target_include_directories(kinematicTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kinematicTest PRIVATE rbdyn)

target_link_libraries(quadrupedKinTest PUBLIC raisim::raisim pthread)
target_include_directories(quadrupedKinTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(quadrupedKinTest PRIVATE rbdyn)