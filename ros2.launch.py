from launch import LaunchDescription
from launch_ros.actions import Node
 

def generate_launch_description():
    
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

   
    turtle_walker = Node(
        package="ROS2_XXXX",
        executable="turtle_walker",
        parameters=[{"turtle_name":"Tom_XXXX"}]
    )
 
    # controller = Node(
    #     package="ROS2_XXXX",
    #     executable="controller",
         
    # )
    # turtle_pos_sub = Node(
    #     package="ROS2_XXXX",
    #     executable="turtle_pos_sub",
         
    # )

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
  
        # turtle_pos_sub,
        turtlesim_node,
        turtle_walker,
        # controller,
        tf_broadcaster1,
        tf_broadcaster2,
        turtle_tf_listener

    ])

    return launch_description
