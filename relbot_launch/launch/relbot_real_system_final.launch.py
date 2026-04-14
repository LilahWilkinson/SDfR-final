from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="relbot_adapter",
            executable="relbot_adapter",
            name="relbot_adapter"
        ),
        Node(
            package="image_processing",
            executable="image_processing_node",
            name="image_processing"
        ),
        Node(
            package="relbot_sequence_controller",
            executable="relbot_sequence_controller",
            name="relbot_sequence_controller"
        )
    ])