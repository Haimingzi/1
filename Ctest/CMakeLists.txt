cmake_minimum_required(VERSION 2.8)

project(test)

set(DEFAULT_CXX_STANDARD 11)

set(OpenCV_DIR /usr/local/opencv3.4.16/lib)#库路径你自己换成你电脑上的

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
 

add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp)
add_executable(main2 ${PROJECT_SOURCE_DIR}/main2.cpp)

target_link_libraries(main  ${OpenCV_LIBS})
target_link_libraries(main2  ${OpenCV_LIBS})
 
