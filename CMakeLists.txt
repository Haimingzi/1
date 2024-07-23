cmake_minimum_required(VERSION 3.5)
project(ROS2_XXXX)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(village_interfaces REQUIRED)
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



#walker
add_executable(turtle_walker src/turtle_walker.cpp)
#walker2
add_executable(turtle_walker2 src/turtle_walker2.cpp)
#controller
add_executable(controller src/TurtleController.cpp)
#pos
add_executable(turtle_pos_sub src/TurtleController.cpp)
#follow
add_executable(follow_turtle src/follow_turtle.cpp)
#tf
add_executable(tf_broadcaster src/tf_broadcaster.cpp)


#walker
ament_target_dependencies(turtle_walker
  rclcpp
  turtlesim
  tf2
  tf2_geometry_msgs
  geometry_msgs
  village_interfaces 
)
#walker2
ament_target_dependencies(turtle_walker2
  rclcpp
  turtlesim
  tf2
  tf2_geometry_msgs
  geometry_msgs
  village_interfaces 
)
#controller
ament_target_dependencies(controller
  rclcpp
  turtlesim
  tf2
  tf2_geometry_msgs
  geometry_msgs
  village_interfaces 
)
#follow
ament_target_dependencies(follow_turtle 
rclcpp
turtlesim
tf2
tf2_geometry_msgs
geometry_msgs
village_interfaces 
)

#pos
ament_target_dependencies(turtle_pos_sub
  rclcpp
  turtlesim
  tf2
  tf2_geometry_msgs
  geometry_msgs
  village_interfaces 
)
#tf
ament_target_dependencies(tf_broadcaster
  rclcpp
  turtlesim
  tf2
  tf2_geometry_msgs
  geometry_msgs
  village_interfaces 
)


#walker
target_include_directories(turtle_walker
  PUBLIC
  $<BUILD_INTERFACE:${tf2_geometry_msgs_INCLUDE_DIRS}>
)
#walker2
target_include_directories(turtle_walker2
  PUBLIC
  $<BUILD_INTERFACE:${tf2_geometry_msgs_INCLUDE_DIRS}>
)
#follow
target_include_directories(follow_turtle
  PUBLIC
  $<BUILD_INTERFACE:${tf2_geometry_msgs_INCLUDE_DIRS}>
)
#controller
target_include_directories(controller
  PUBLIC
  $<BUILD_INTERFACE:${tf2_geometry_msgs_INCLUDE_DIRS}>
)
#tf
target_include_directories(tf_broadcaster
  PUBLIC
  $<BUILD_INTERFACE:${tf2_geometry_msgs_INCLUDE_DIRS}>
)



# 设置ROS 2执行文件
ament_export_dependencies(turtlesim geometry_msgs)

# 安装可执行文件
install(TARGETS
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



#walker
install(TARGETS
  turtle_walker
  DESTINATION lib/${PROJECT_NAME}
)
#walker2
install(TARGETS
  turtle_walker2
  DESTINATION lib/${PROJECT_NAME}
)
#controller
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)
#follow
install(TARGETS
  follow_turtle
  DESTINATION lib/${PROJECT_NAME}
)
#pos
install(TARGETS
  turtle_pos_sub
  DESTINATION lib/${PROJECT_NAME}
)
#tf
install(TARGETS
  tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)

ament_package()
