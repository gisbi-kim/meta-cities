cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iterative_closest_point)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.2 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include # meta_cities
)

# message(${CMAKE_CURRENT_SOURCE_DIR}/dd)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iterative_closest_point main.cpp)

target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})
