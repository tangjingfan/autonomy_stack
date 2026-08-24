"""Multi-robot CMU stack: one Gazebo world, namespaced robot_i groups."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


WORLD_PRESETS = {
    'garage': {
        'checkTerrainConn': 'true',
        'vehicleHeight': '0.75',
        'lidar_urdf': 'urdf/lidar.urdf.xacro',
        'robot_urdf': 'urdf/robot.sdf',
        'adjustZ': 'true',
        'adjustIncl': 'true',
        'groundHeightThre': '0.1',
        'local_planner': {},
    },
    'indoor': {
        'checkTerrainConn': 'false',
        'vehicleHeight': '0.75',
        'lidar_urdf': 'urdf/lidar.urdf.xacro',
        'robot_urdf': 'urdf/robot.sdf',
        'adjustZ': 'true',
        'adjustIncl': 'true',
        'groundHeightThre': '0.1',
        'local_planner': {},
    },
    'forest': {
        'checkTerrainConn': 'true',
        'vehicleHeight': '0.75',
        'lidar_urdf': 'urdf/lidar.urdf.xacro',
        'robot_urdf': 'urdf/robot.sdf',
        'adjustZ': 'true',
        'adjustIncl': 'true',
        'groundHeightThre': '0.1',
        'local_planner': {},
    },
    'campus': {
        'checkTerrainConn': 'false',
        'vehicleHeight': '0.75',
        'lidar_urdf': 'urdf/lidar.urdf.xacro',
        'robot_urdf': 'urdf/robot.sdf',
        'adjustZ': 'true',
        'adjustIncl': 'true',
        'groundHeightThre': '0.1',
        'local_planner': {},
    },
    'tunnel': {
        'checkTerrainConn': 'false',
        'vehicleHeight': '0.75',
        'lidar_urdf': 'urdf/lidar.urdf.xacro',
        'robot_urdf': 'urdf/robot.sdf',
        'adjustZ': 'true',
        'adjustIncl': 'true',
        'groundHeightThre': '0.1',
        'local_planner': {},
    },
    'matterport': {
        'checkTerrainConn': 'false',
        'vehicleHeight': '0.5',
        'lidar_urdf': 'urdf_mp3d/lidar.urdf.xacro',
        'robot_urdf': 'urdf_mp3d/robot.sdf',
        'adjustZ': 'false',
        'adjustIncl': 'false',
        'groundHeightThre': '0.25',
        'local_planner': {
            'vehicleLength': '0.1',
            'vehicleWidth': '0.1',
            'terrainVoxelSize': '0.1',
            'useTerrainAnalysis': 'true',
            'twoWayDrive': 'false',
            'adjacentRange': '2.0',
            'obstacleHeightThre': '0.65',
            'minRelZ': '-0.25',
            'pathScale': '0.5',
            'minPathScale': '0.3',
            'minPathRange': '0.5',
            'slowDwnDisThre': '0.3',
            'useRgbdCamera': 'false',
            'maxSpeed': '2.0',
        },
    },
}


def _parse_csv_floats(text, n, spacing, axis='x'):
    values = [float(tok.strip()) for tok in text.split(',') if tok.strip()]
    while len(values) < n:
        if axis == 'x':
            values.append(spacing * len(values))
        else:
            values.append(0.0)
    return values[:n]


def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    n_robots = max(1, int(LaunchConfiguration('n_robots').perform(context)))
    gazebo_gui = LaunchConfiguration('gazebo_gui').perform(context)
    start_rviz = LaunchConfiguration('start_rviz').perform(context)
    spacing = float(LaunchConfiguration('spawn_spacing').perform(context))
    xs = _parse_csv_floats(
        LaunchConfiguration('vehicleX_list').perform(context), n_robots, spacing, 'x')
    ys = _parse_csv_floats(
        LaunchConfiguration('vehicleY_list').perform(context), n_robots, spacing, 'y')
    yaws = _parse_csv_floats(
        LaunchConfiguration('vehicleYaw_list').perform(context), n_robots, 0.0, 'y')

    preset = WORLD_PRESETS.get(world_name, WORLD_PRESETS['garage'])
    pkg_sim = get_package_share_directory('vehicle_simulator')
    world_file = os.path.join(pkg_sim, 'world', world_name + '.world')

    actions = []
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_file,
            'gui': gazebo_gui,
            'verbose': 'false',
        }.items(),
    ))

    for i in range(n_robots):
        ns = f'robot_{i}'
        sensor_frame = f'{ns}/sensor'
        vehicle_frame = f'{ns}/vehicle'
        camera_frame = f'{ns}/camera'
        lp_args = {
            'cameraOffsetZ': LaunchConfiguration('cameraOffsetZ'),
            'goalX': str(xs[i]),
            'goalY': str(ys[i]),
            'sensor_frame': sensor_frame,
            'vehicle_frame': vehicle_frame,
            'camera_frame': camera_frame,
        }
        lp_args.update(preset.get('local_planner', {}))

        robot_group = GroupAction([
            PushRosNamespace(ns),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('local_planner'),
                    'launch', 'local_planner.launch')),
                launch_arguments=lp_args.items(),
            ),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('terrain_analysis'),
                    'launch', 'terrain_analysis.launch')),
            ),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('terrain_analysis_ext'),
                    'launch', 'terrain_analysis_ext.launch')),
                launch_arguments={
                    'checkTerrainConn': preset['checkTerrainConn'],
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg_sim, 'launch', 'vehicle_simulator.launch')),
                launch_arguments={
                    'world_name': world_name,
                    'vehicleHeight': preset['vehicleHeight'],
                    'cameraOffsetZ': LaunchConfiguration('cameraOffsetZ'),
                    'vehicleX': str(xs[i]),
                    'vehicleY': str(ys[i]),
                    'terrainZ': LaunchConfiguration('terrainZ'),
                    'vehicleYaw': str(yaws[i]),
                    'gui': gazebo_gui,
                    'start_gazebo': 'false',
                    'lidar_urdf': preset['lidar_urdf'],
                    'robot_urdf': preset['robot_urdf'],
                    'adjustZ': preset['adjustZ'],
                    'adjustIncl': preset['adjustIncl'],
                    'groundHeightThre': preset['groundHeightThre'],
                    'robot_model_name': f'robot_{i}',
                    'lidar_model_name': f'lidar_{i}',
                    'camera_model_name': f'camera_{i}',
                    'mapFrame': 'map',
                    'sensorFrame': sensor_frame,
                    'scan_topic': f'/{ns}/velodyne_points',
                    'tf_prefix': ns,
                    'sensor_name': f'velodyne_{i}',
                    'parent_link': f'{ns}/lidar',
                    'robot_ns': ns,
                    'camera_frame': camera_frame,
                }.items(),
            ),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('sensor_scan_generation'),
                    'launch', 'sensor_scan_generation.launch')),
                launch_arguments={
                    'map_frame': 'map',
                    'sensor_at_scan_frame': f'{ns}/sensor_at_scan',
                }.items(),
            ),
        ])
        actions.append(robot_group)

    actions.append(IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('visualization_tools'),
            'launch', 'visualization_tools.launch')),
        launch_arguments={
            'world_name': world_name,
            'odom_topic': '/robot_0/state_estimation',
            'scan_topic': '/robot_0/registered_scan',
        }.items(),
    ))

    actions.append(Node(
        package='joy',
        executable='joy_node',
        name='ps3_joy',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joy')),
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.12,
            'autorepeat_rate': 0.0,
        }],
    ))

    rviz_config_file = os.path.join(pkg_sim, 'rviz', 'vehicle_simulator.rviz')
    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(start_rviz),
    )
    actions.append(TimerAction(period=8.0, actions=[start_rviz_node]))
    return actions


def generate_launch_description():
    mesh_dir = os.path.join(get_package_share_directory('vehicle_simulator'), 'mesh')
    model_path = mesh_dir
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing:
        model_path = mesh_dir + os.pathsep + existing
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        DeclareLaunchArgument('world_name', default_value='garage'),
        DeclareLaunchArgument('n_robots', default_value='1'),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0'),
        DeclareLaunchArgument('terrainZ', default_value='0.0'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('spawn_spacing', default_value='3.0'),
        DeclareLaunchArgument('vehicleX_list', default_value='0.0,3.0,6.0,9.0'),
        DeclareLaunchArgument('vehicleY_list', default_value='0.0,0.0,0.0,0.0'),
        DeclareLaunchArgument('vehicleYaw_list', default_value='0.0,0.0,0.0,0.0'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
