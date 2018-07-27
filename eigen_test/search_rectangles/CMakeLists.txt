cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extract_indices)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (search_rectangles search_rectangles.cpp)
target_link_libraries (search_rectangles ${PCL_LIBRARIES})
