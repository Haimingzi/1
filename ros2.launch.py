from launch import LaunchDescription
from launch_ros.actions import Node
 

def generate_launch_description():
    turtle_walker2 = Node(
        package="ROS2_XXXX",
        executable="turtle_walker2"
    )
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[{"name":"Jerry_XXXX","x":5.5,"y":5.5,"background_g":255,"background_r":0,"background_b":0}]
    )

   
    turtle_walker = Node(
        package="ROS2_XXXX",
        executable="turtle_walker",
        parameters=[{"turtle_name":"Tom_XXXX"}]
    )
 
    controller = Node(
        package="ROS2_XXXX",
        executable="controller",
         
    )
 

    #分别广播两只乌龟相对于world的坐标变换

    tf_broadcaster1 = Node(
        package="ROS2_XXXX",
        executable="tf_broadcaster",
        name="tf_broadcaster1",
        parameters=[{"turtle":"turtle1"}]

    )

    tf_broadcaster2 = Node(
        package="ROS2_XXXX",
        executable="tf_broadcaster",
        name="tf_broadcaster2",
        parameters=[{"turtle":"Tom_XXXX"}]
    )
    

    turtle_tf_listener=Node(
        package="ROS2_XXXX",
        executable="turtle_tf_listener",
        parameters=[{"father_frame":"Tom_XXXX","child_frame":"turtle1"}]
    )

    launch_description = LaunchDescription([
  
        turtlesim_node,
        turtle_walker,
        controller,
        tf_broadcaster1,
        tf_broadcaster2,
        turtle_tf_listener

    ])

    return launch_description
