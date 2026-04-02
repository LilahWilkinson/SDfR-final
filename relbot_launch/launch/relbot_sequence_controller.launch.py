from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="relbot_simulator",
            executable="relbot_simulator",
            name="relbot_simulator"
        ),
        Node(
            package="relbot2turtlesim",
            executable="relbot2turtlesim",
            name="relbot2turtlesim"
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim"
        ),
        Node(
            package="relbot_sequence_controller",
            executable="relbot_sequence_controller",
            name="relbot_sequence_controller"
        )
    ])