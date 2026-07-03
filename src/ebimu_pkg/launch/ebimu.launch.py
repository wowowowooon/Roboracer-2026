#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    accel_in_g = LaunchConfiguration("accel_in_g")

    return LaunchDescription([

        DeclareLaunchArgument(
            "port",
            default_value="auto",
            description="EBIMU 시리얼 포트 (auto, /dev/ttyTHS1, /dev/ttyUSB0 등)",
        ),
        DeclareLaunchArgument(
            "baud",
            default_value="115200",
            description="EBIMU 시리얼 통신 속도 (115200 등)",
        ),
        DeclareLaunchArgument(
            "accel_in_g",
            default_value="true",
            description="EBIMU accel 출력이 g 단위면 true, m/s^2면 false",
        ),

        Node(
            package="ebimu_pkg",
            executable="ebimu_driver",
            name="ebimu_driver",
            parameters=[{
                "port": port,
                "baud": baud,
                "accel_in_g": accel_in_g,
            }],
            output="log",
        ),
    ])

