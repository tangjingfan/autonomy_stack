import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_world_wrapper(world_name):
    pkg = get_package_share_directory('vehicle_simulator')
    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='1'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('spawn_spacing', default_value='3.0'),
        DeclareLaunchArgument('vehicleX_list', default_value='0.0,3.0,6.0,9.0'),
        DeclareLaunchArgument('vehicleY_list', default_value='0.0,0.0,0.0,0.0'),
        DeclareLaunchArgument('vehicleYaw_list', default_value='0.0,0.0,0.0,0.0'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'system.launch.py')),
            launch_arguments={
                'world_name': world_name,
                'n_robots': LaunchConfiguration('n_robots'),
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'start_rviz': LaunchConfiguration('start_rviz'),
                'spawn_spacing': LaunchConfiguration('spawn_spacing'),
                'vehicleX_list': LaunchConfiguration('vehicleX_list'),
                'vehicleY_list': LaunchConfiguration('vehicleY_list'),
                'vehicleYaw_list': LaunchConfiguration('vehicleYaw_list'),
                'use_joy': LaunchConfiguration('use_joy'),
            }.items(),
        ),
    ])
