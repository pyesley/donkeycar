import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Paths to standard Jazzy packages
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # 2. Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 3. SLAM Toolbox (Async Mode)
    # This node creates the map from Lidar scans and handles the broken encoder issue
    # by using scan-matching (Lidar Odometry).
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Navigation2 Bringup
    # We launch navigation without AMCL (because SLAM is doing localization)
    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(os.getcwd(), 'nav_params.yaml')  # We will generate this next
        }.items()
    )

    # 5. Our Hardware Bridge
    hardware_bridge_node = Node(
        package='your_robot_package',  # Replace with your actual package name
        executable='hardware_bridge.py',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_navigation_cmd)
    ld.add_action(hardware_bridge_node)

    return ld