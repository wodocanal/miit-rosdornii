from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="gnss_resilient_demo",
                executable="demo_gui",
                name="gnss_resilient_demo_gui",
                output="screen",
            )
        ]
    )
