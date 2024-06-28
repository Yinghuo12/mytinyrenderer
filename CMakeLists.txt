cmake_minimum_required(VERSION 3.1)
project(mytinyrenderer)             #项目名程

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_SUB)   #子目录
# aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRC_CUR)     #当前目录
# file(GLOB SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)        #包含头文件目录
set(CMAKE_CXX_STANDARD 11)


# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/output)
add_executable(tinyrenderer ${SRC_SUB} ${SRC_CUR} main.cpp)     #生成可执行文件
