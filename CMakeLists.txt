cmake_minimum_required(VERSION 3.10)
project(rbdyn LANGUAGES CXX)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
find_package(TinyXML2 REQUIRED)

add_subdirectory(examples)

include_directories(${EIGEN3_INCLUDE_DIRS} "/usr/local/include")
link_directories("/usr/local/lib")

add_library(rbdyn SHARED src/Dynamics.cpp)
target_include_directories(rbdyn PUBLIC include)
target_link_libraries(rbdyn PUBLIC tinyxml2)

install(TARGETS rbdyn DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/rbdyn)