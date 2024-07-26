import os

from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    
    camera = Node(
            package='truck',
            executable='usb_camera.py',
            name='usb_camera_node',
            output='screen',
        )
    
    return LaunchDescription([
        camera
    ])
