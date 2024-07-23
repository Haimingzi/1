from launch import LaunchDescription
from launch_ros.actions import Node
 

def generate_launch_description():
    
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        output="screen",

        
    )

   
    turtle_walker = Node(
        package="ROS2_XXXX",
        executable="turtle_walker",
         
    )
 
    controller = Node(
        package="ROS2_XXXX",
        executable="controller",
         
    )
    turtle_pos_sub = Node(
        package="ROS2_XXXX",
        executable="turtle_pos_sub",
         
    )
    
    launch_description = LaunchDescription([
        turtle_walker,
        controller,
        turtle_pos_sub,
        turtlesim_node
    ])

    return launch_description