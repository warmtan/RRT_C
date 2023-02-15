# 支持cmake 最小版崩本
cmake_minimum_required(VERSION 3.10) 

project(path_planning)

set(SOURCE
    common/map.cc
    a_star/a_star.cc
)

add_executable(path_planning ${SOURCE})

# 找到OpenCV的相关包
find_package(OpenCV REQUIRED) 
# 
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(path_planning ${OpenCV_LIBS})

target_include_directories(path_planning
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/common
)
