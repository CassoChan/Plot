cmake_minimum_required (VERSION 3.15)
project (plot)

SET(CMAKE_CXX_STANDARD 14)
#SET(CMAKE_C_STANDARD 11)


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

add_executable(plot main.cpp)

# python的头文件 PYTHON_INCLUDE_DIRS
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
# python的库文件 PYTHON_LIBRARIES   
# 解决报错： fatal error: Python.h: 没有那个文件或目录
target_link_libraries(plot ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})