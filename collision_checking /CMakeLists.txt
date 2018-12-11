cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(crop_box)

find_package(PCL 1.2 REQUIRED)

include_directories(
  include
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (collision_checker src/test.cpp src/collision_checking.cpp)
target_link_libraries (collision_checker ${PCL_LIBRARIES})
