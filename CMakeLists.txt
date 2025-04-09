cmake_minimum_required(VERSION 3.10)
project(rbdyn LANGUAGES CXX)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
find_package(TinyXML2 REQUIRED)


add_library(rbdyn SHARED src/Dynamics.cpp 
                         src/Kinematics.cpp 
                         src/orientTools.cpp 
                         src/Spatial.cpp 
                         src/SpatialInertia.cpp)

target_include_directories(rbdyn PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                        $<INSTALL_INTERFACE:include/rbdyn>
                                        ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(rbdyn PUBLIC tinyxml2)

add_subdirectory(examples)

include(GNUInstallDirs)

install(TARGETS rbdyn
    EXPORT rbdynTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rbdyn)

install(EXPORT rbdynTargets
    FILE rbdynConfig.cmake
    NAMESPACE rbdyn::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rbdyn
)

