"""
PiDog ROS2 Launch File

Launches the unified PiDog node with camera and audio streaming.

Usage:
    ros2 launch pidog_ros pidog.launch.py

With custom parameters:
    ros2 launch pidog_ros pidog.launch.py camera_width:=1280 camera_height:=720
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera image width'
    )

    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera image height'
    )

    camera_framerate_arg = DeclareLaunchArgument(
        'camera_framerate',
        default_value='30',
        description='Camera framerate'
    )

    audio_capture_device_arg = DeclareLaunchArgument(
        'audio_capture_device',
        default_value='plughw:2,0',
        description='ALSA capture device for microphone'
    )

    audio_playback_device_arg = DeclareLaunchArgument(
        'audio_playback_device',
        default_value='plughw:2,0',
        description='ALSA playback device for speaker'
    )

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera streaming'
    )

    enable_audio_arg = DeclareLaunchArgument(
        'enable_audio',
        default_value='true',
        description='Enable audio streaming'
    )

    # PiDog node
    pidog_node = Node(
        package='pidog_ros',
        executable='pidog_node',
        name='pidog_node',
        output='screen',
        parameters=[{
            'camera.width': LaunchConfiguration('camera_width'),
            'camera.height': LaunchConfiguration('camera_height'),
            'camera.framerate': LaunchConfiguration('camera_framerate'),
            'camera.frame_id': 'pidog_camera_frame',
            'camera.jpeg_quality': 80,
            'audio.capture_device': LaunchConfiguration('audio_capture_device'),
            'audio.playback_device': LaunchConfiguration('audio_playback_device'),
            'audio.enable_capture': True,
            'audio.enable_playback': True,
            'audio.capture_volume': 80,
            'audio.playback_volume': 80,
        }]
    )

    return LaunchDescription([
        camera_width_arg,
        camera_height_arg,
        camera_framerate_arg,
        audio_capture_device_arg,
        audio_playback_device_arg,
        enable_camera_arg,
        enable_audio_arg,
        pidog_node,
    ])
