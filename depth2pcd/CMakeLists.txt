cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(depth2pcd)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(~/Repos/SUNCGtoolbox/gaps/pkgs)
link_directories(~/Repos/SUNCGtoolbox/gaps/lib/x86_64)
set(GAPS_LIBRARIES R3Graphics R3Shapes R2Shapes RNBasics png jpeg)

add_executable (depth2pcd depth2pcd.cpp)
target_link_libraries (depth2pcd ${PCL_LIBRARIES} ${GAPS_LIBRARIES} -lGL -lGLU)
add_executable (depth2pcd2 depth2pcd2.cpp)
target_link_libraries (depth2pcd2 ${PCL_LIBRARIES} ${GAPS_LIBRARIES} -lGL -lGLU)
add_executable (fuse_depths fuse_depths.cpp)
target_link_libraries (fuse_depths ${PCL_LIBRARIES} ${GAPS_LIBRARIES} -lGL -lGLU)