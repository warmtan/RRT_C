# 支持cmake最小版本
cmake_minimum_required(VERSION 2.8.3)

# 项目名称
project(rrt)

# 添加项目包
find_package(OpenCV REQUIRED)

# 该命令将指定目录添加到编译器用于搜索包含头文件的目录路径中去。
include_directories(include
   ${catkin_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
)

# set命令可以设置普通变量、缓存条目、环境变量三种变量的值，分别对应以下三种命令格式。
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(SRC_LIST main.cpp rrt.h rrt.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# 三行比较关键，表示最终要生成的elf文件的名字叫main，使用的源文件是main.c
add_executable(past ${SRC_LIST})

# target_link_libraries命令用于指定链接给定目标和/或其依赖项时要使用的库或标志。
target_link_libraries(past ${catkin_LIBRARIES} ${OpenCV_LIBS})


# cmake_minimum_required (VERSION 2.8)

# project (demo)

# set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# aux_source_directory (src SRC_LIST)

# include_directories (include)

# add_executable (main ${SRC_LIST})
