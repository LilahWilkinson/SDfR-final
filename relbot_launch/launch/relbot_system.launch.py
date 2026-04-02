from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node


def generate_launch_description():
    # use_twist_cmd_arg = DeclareLaunchArgument(
    #     name="use_twist_cmd",
    #     default_value="True",
    #     description="Whether to use Twist command mode. If set to false, uses individual motor command mode.",
    # )

    relbot_simulator = Node(
        package="relbot_simulator",
        executable="relbot_simulator",
        name="relbot_sim",
        parameters=[{"use_twist_cmd": True}]
    )

    relbot2turtlesim = Node(
        package="relbot2turtlesim",
        executable="relbot2turtlesim",
        name="pose_to_TS"
    )

    relbot_turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="robotturtle"
    )

    return LaunchDescription([
        relbot_simulator,
        relbot2turtlesim,
        relbot_turtle
    ])
