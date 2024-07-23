from launch import LaunchDescription
from launch_ros.actions import Node
 

def generate_launch_description():
    
 

   
    turtle_walker2 = Node(
        package="ROS2_XXXX",
        executable="turtle_walker2",
         
    )
 
    follow_turtle = Node(
        package="ROS2_XXXX",
        executable="follow_turtle",
         
    )
     
    
    launch_description = LaunchDescription([
        turtle_walker2,
        follow_turtle,
    ])

    return launch_description