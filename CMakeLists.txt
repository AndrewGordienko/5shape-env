cmake_minimum_required(VERSION 3.14)
project(cmake_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(SDL2 REQUIRED)
find_package(Box2D REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS} ${Box2D_INCLUDE_DIRS})

# Compile your sources and link with found libraries
pybind11_add_module(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE ${SDL2_LIBRARIES} Box2D pybind11::embed)
