cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(findobjects)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(~/Repos/SUNCGtoolbox/gaps/pkgs)
link_directories(~/Repos/SUNCGtoolbox/gaps/lib/x86_64)
set(GAPS_LIBRARIES R3Graphics R3Shapes R2Shapes RNBasics png jpeg)

add_executable (findobjects findobjects.cpp)
target_link_libraries (findobjects ${PCL_LIBRARIES} ${GAPS_LIBRARIES})