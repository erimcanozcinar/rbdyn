cmake_minimum_required(VERSION 3.10)
project(kinematicTest LANGUAGES CXX)

add_executable(kinematicTest src/main.cpp)
add_executable(quadrupedKinTest src/main_quadruped.cpp)
add_executable(floatKinTest src/main_float.cpp)

target_link_libraries(kinematicTest PUBLIC raisim::raisim pthread)
target_include_directories(kinematicTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(kinematicTest PRIVATE rbdyn)

target_link_libraries(quadrupedKinTest PUBLIC raisim::raisim pthread)
target_include_directories(quadrupedKinTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(quadrupedKinTest PRIVATE rbdyn)

target_link_libraries(floatKinTest PUBLIC raisim::raisim pthread)
target_include_directories(floatKinTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(floatKinTest PRIVATE rbdyn)