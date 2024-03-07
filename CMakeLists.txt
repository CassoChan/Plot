cmake_minimum_required (VERSION 3.15)
project (plot)

SET(CMAKE_CXX_STANDARD 14)
#SET(CMAKE_C_STANDARD 11)


set(ALL_SRCS "minimal.cpp")

add_executable(plot main.cpp)

# cmake方式依赖Python头文件和库文件的配置

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(PythonLibs REQUIRED)

# python的头文件 PYTHON_INCLUDE_DIRS
include_directories(${PYTHON_INCLUDE_DIRS})
# python的库文件 PYTHON_LIBRARIES   
# 解决报错： fatal error: Python.h: 没有那个文件或目录
target_link_libraries(plot ${PYTHON_LIBRARIES})