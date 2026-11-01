from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    description_file = LaunchConfiguration('description_file')
    world_file = LaunchConfiguration('world_file')

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_simulation_gz'),
                'launch',
                'ur_sim_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': launch_rviz,
            'gazebo_gui': gazebo_gui,
            'description_file': description_file,
            'world_file': world_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument(
            'description_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('manipulator_ros2'), 'urdf', 'ur5e_table.urdf.xacro'
            ]),
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('manipulator_ros2'), 'worlds', 'table.sdf'
            ]),
        ),
        ur_sim,
    ])