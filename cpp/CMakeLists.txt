cmake_minimum_required(VERSION 3.16)

project(ru_racer_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ru_racer
  src/main.cpp
  src/bike_model.cpp
  src/occupancy_grid.cpp
  src/rrt_star.cpp
  src/sparse_rrt_bike.cpp
  src/nmpc.cpp
)

target_include_directories(ru_racer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (MSVC)
  target_compile_options(ru_racer PRIVATE /W4 /permissive-)
else()
  target_compile_options(ru_racer PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

