cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(scn2pcd)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(~/Repos/SUNCGtoolbox/gaps/pkgs)
link_directories(~/Repos/SUNCGtoolbox/gaps/lib/x86_64)
set(GAPS_LIBRARIES R3Graphics R3Shapes R2Shapes RNBasics png jpeg)

add_executable (scn2pcd scn2pcd.cpp)
target_link_libraries (scn2pcd ${GAPS_LIBRARIES} ${PCL_LIBRARIES} -lGL -lGLU)
add_executable (scn2poses scn2poses.cpp)
target_link_libraries (scn2poses ${GAPS_LIBRARIES} -lGL -lGLU)