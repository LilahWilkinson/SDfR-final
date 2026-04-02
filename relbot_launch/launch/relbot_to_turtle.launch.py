from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="relbot_simulator",
            executable="relbot_simulator",
            name="relbot_sim"
        ),
        Node(
            package="relbot2turtlesim",
            executable="relbot2turtlesim",
            name="relbot2TS"
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim"
        )
    ])