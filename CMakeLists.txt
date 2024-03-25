cmake_minimum_required (VERSION 3.15)
project (plot2)

SET(CMAKE_CXX_STANDARD 17)

set(ALL_SRCS "minimal.cpp")

# OpenCV
find_package(OpenCV REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# PCL 1.10
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# matplotlibcpp17
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})


add_executable(plot2 main.cpp)
target_include_directories(plot2 PUBLIC ${Python3_INCLUDE_DIRS})
target_link_libraries(plot2 ${Python3_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} yaml-cpp pybind11::embed matplotlibcpp17::matplotlibcpp17)