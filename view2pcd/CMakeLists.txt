cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(view2pcd)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(~/Repos/SUNCGtoolbox/gaps/pkgs)
link_directories(~/Repos/SUNCGtoolbox/gaps/lib/x86_64)
set(GAPS_LIBRARIES R3Graphics R3Shapes R2Shapes RNBasics png jpeg)

add_executable (view2pcd view2pcd.cpp)
target_link_libraries (view2pcd ${GAPS_LIBRARIES} ${PCL_LIBRARIES} -lGL -lGLU)
add_executable (pcd2cameracoord pcd2cameracoord.cpp)
target_link_libraries (pcd2cameracoord ${GAPS_LIBRARIES} ${PCL_LIBRARIES})