import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    Service = Node(
        package="ros_basic",
        executable="review1.py",
        name="simple_service_server",
        output="screen",
    )

    return LaunchDescription([
        Service
    ])