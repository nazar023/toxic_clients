import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    custom_way_arg = DeclareLaunchArgument(
            'custom_way',
        )

    custom_way = LaunchConfiguration('custom_way')

    return LaunchDescription([
        custom_way_arg,
        OpaqueFunction(function=get_custom_nodes, args=[custom_way]),
    ])

def get_config(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def get_custom_nodes(context, custom_way):
    module_siyi_pkg_dir = get_package_share_directory("toxic_clients")
    custom_way_str = custom_way.perform(context)

    toxic_config = get_config(f"{module_siyi_pkg_dir}/config/{custom_way_str}")

    return [
        Node(
            package='toxic_clients',
            executable='custom_client',
            parameters=[toxic_config],
        ),
    ]
