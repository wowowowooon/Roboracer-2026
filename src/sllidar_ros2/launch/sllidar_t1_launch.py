#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    channel_type = LaunchConfiguration('channel_type', default='udp')
    udp_ip = LaunchConfiguration('udp_ip', default='192.168.11.2')
    udp_port = LaunchConfiguration('udp_port', default='8089')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='false')
    angle_offset = LaunchConfiguration('angle_offset', default='0.0')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    scan_frequency = LaunchConfiguration('scan_frequency', default='40.0')

    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'udp_ip',
            default_value=udp_ip,
            description='Specifying udp ip to connected lidar'),

        DeclareLaunchArgument(
            'udp_port',
            default_value=udp_port,
            description='Specifying udp port to connected lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying serial port of lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying serial baudrate of lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'angle_offset',
            default_value=angle_offset,
            description='Scan angle offset in radians'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        DeclareLaunchArgument(
            'scan_frequency',
            default_value=scan_frequency,
            description='Specifying scan frequency of lidar'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': channel_type, 
                         'udp_ip': udp_ip,
                         'udp_port': ParameterValue(udp_port, value_type=int),
                         'serial_port': serial_port,
                         'serial_baudrate': ParameterValue(serial_baudrate, value_type=int),
                         'frame_id': frame_id,
                         'inverted': ParameterValue(inverted, value_type=bool),
                         'angle_compensate': ParameterValue(angle_compensate, value_type=bool),
                         'angle_offset': ParameterValue(angle_offset, value_type=float),
                         'scan_mode': scan_mode,
                         'scan_frequency': ParameterValue(scan_frequency, value_type=float)}],
            output='log'),
    ])
