#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/usb_cam/image_raw', description='Input image topic'),
        DeclareLaunchArgument('output_topic', default_value='/image_conversion/output_image', description='Output image topic'),
        
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{'video_device': '/dev/video0'}],
        ),
        
        Node(
            package='image_converter',
            executable='image_conversion_node',
            name='image_conversion',
            output='screen',
            parameters=[{'input_topic': LaunchConfiguration('input_topic'), 'output_topic': LaunchConfiguration('output_topic')}],
            remappings=[('/usb_cam/image_raw', LaunchConfiguration('input_topic')), 
                        ('/image_conversion/output_image', LaunchConfiguration('output_topic'))]
        ),
    ])

