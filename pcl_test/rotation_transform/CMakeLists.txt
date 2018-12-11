cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rotation_transform)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (rotation_transform rotation_transform.cpp)
target_link_libraries (rotation_transform ${PCL_LIBRARIES})
