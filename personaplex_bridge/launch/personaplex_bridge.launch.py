"""
PersonaPlex Bridge Launch File

Launches the bridge node that connects PiDog audio to PersonaPlex.
Run this on the desktop machine with the RTX GPU.

Usage:
    ros2 launch personaplex_bridge personaplex_bridge.launch.py

With custom parameters:
    ros2 launch personaplex_bridge personaplex_bridge.launch.py server_host:=192.168.1.100
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    server_host_arg = DeclareLaunchArgument(
        'server_host',
        default_value='localhost',
        description='PersonaPlex server hostname'
    )

    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='8998',
        description='PersonaPlex server port'
    )

    use_ssl_arg = DeclareLaunchArgument(
        'use_ssl',
        default_value='true',
        description='Use SSL/TLS connection'
    )

    voice_arg = DeclareLaunchArgument(
        'voice',
        default_value='NATF2',
        description='PersonaPlex voice prompt (NATF0-3, NATM0-3, VARF0-4, VARM0-4)'
    )

    system_prompt_arg = DeclareLaunchArgument(
        'system_prompt',
        default_value='You are a helpful assistant on a robot dog called PiDog. Keep responses brief and friendly.',
        description='PersonaPlex system/role prompt'
    )

    # PersonaPlex bridge node
    bridge_node = Node(
        package='personaplex_bridge',
        executable='personaplex_bridge_node',
        name='personaplex_bridge_node',
        output='screen',
        parameters=[{
            'server.host': LaunchConfiguration('server_host'),
            'server.port': LaunchConfiguration('server_port'),
            'server.use_ssl': LaunchConfiguration('use_ssl'),
            'personaplex.voice': LaunchConfiguration('voice'),
            'personaplex.system_prompt': LaunchConfiguration('system_prompt'),
        }]
    )

    return LaunchDescription([
        server_host_arg,
        server_port_arg,
        use_ssl_arg,
        voice_arg,
        system_prompt_arg,
        bridge_node,
    ])
