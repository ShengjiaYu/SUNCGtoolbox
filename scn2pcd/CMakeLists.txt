cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(scn2pcd)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(/usr1/home/wyuan1/Documents/SUNCGtoolbox/gaps/pkgs)
link_directories(/usr1/home/wyuan1/Documents/SUNCGtoolbox/gaps/lib/x86_64)
set(GAPS_LIBRARIES R3Graphics R3Shapes R2Shapes RNBasics)

add_executable (scn2pcd scn2pcd.cpp)
target_link_libraries (scn2pcd ${GAPS_LIBRARIES} ${PCL_LIBRARIES})
add_executable (pcd2cameracoord pcd2cameracoord.cpp)
target_link_libraries (pcd2cameracoord ${GAPS_LIBRARIES} ${PCL_LIBRARIES})