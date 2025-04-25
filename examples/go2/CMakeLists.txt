cmake_minimum_required(VERSION 3.10)
project(go2Test LANGUAGES CXX)

add_executable(go2Test src/main.cpp)
add_executable(go2Testdyn src/main_dyn.cpp)
add_executable(go2TestdynGazebo src/main_dyn_gazebo.cpp)

target_link_libraries(go2Test PUBLIC raisim::raisim pthread)
target_include_directories(go2Test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(go2Test PRIVATE rbdyn)

target_link_libraries(go2Testdyn PUBLIC raisim::raisim pthread)
target_include_directories(go2Testdyn PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(go2Testdyn PRIVATE rbdyn)

target_link_libraries(go2TestdynGazebo PUBLIC raisim::raisim pthread)
target_include_directories(go2TestdynGazebo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(go2TestdynGazebo PRIVATE rbdyn)