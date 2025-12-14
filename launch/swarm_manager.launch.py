#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    image_topic = LaunchConfiguration('image_source_topic')
    compass_topic = LaunchConfiguration('compass_topic')
    gps_topic = LaunchConfiguration('gps_topic')
    mag_topic = LaunchConfiguration('mag_topic')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out_topic')

    return LaunchDescription([

        DeclareLaunchArgument(
            'image_source_topic',
            default_value='/camera/image_raw'
        ),

        DeclareLaunchArgument(
            'compass_topic',
            default_value='/imu/compass'
        ),

        DeclareLaunchArgument(
            'gps_topic',
            default_value='/gps/fix'
        ),

        DeclareLaunchArgument(
            'mag_topic',
            default_value='/imu/mag'
        ),

        DeclareLaunchArgument(
            'cmd_vel_out_topic',
            default_value='/cmd_vel'
        ),

        Node(
            package='articubot_three',
            executable='swarm_node_manager.py',
            name='swarm_node_manager',
            output='screen',
            parameters=[{
                'image_source_topic': image_topic,
                'compass_topic': compass_topic,
                'gps_topic': gps_topic,
                'mag_topic': mag_topic,
                'cmd_vel_out_topic': cmd_vel_out,
            }]
        )
    ])
