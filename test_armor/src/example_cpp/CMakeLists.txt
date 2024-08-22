cmake_minimum_required(VERSION 3.5)
project(example_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR /usr/local/opencv3.4.16/lib)#库路径你自己换成你电脑上的
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



 
add_executable(main1 src/main1.cpp)

target_link_libraries(main1 ${OpenCV_LIBS})
 
ament_target_dependencies(main1
  rclcpp
  turtlesim
  tf2
  tf2_geometry_msgs
  geometry_msgs
 
)
 


#walker
target_include_directories(main1
  PUBLIC
  $<BUILD_INTERFACE:${tf2_geometry_msgs_INCLUDE_DIRS}>
)
  
 
# 安装可执行文件
install(TARGETS
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



install(TARGETS
  main1
  DESTINATION lib/${PROJECT_NAME}
)
 

ament_package()





