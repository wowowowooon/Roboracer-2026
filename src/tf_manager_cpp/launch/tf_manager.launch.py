
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_wheel_odom_tf = LaunchConfiguration("use_wheel_odom_tf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_wheel_odom_tf",
            default_value="true",
            description="Publish wheel odom and odom->base_link TF",
        ),

        Node(
            package="tf_manager_cpp",
            executable="sensor_static_tf",
            name="sensor_static_tf"
        ),

        Node(
            package="tf_manager_cpp",
            executable="wheel_odom_tf",
            name="wheel_odom_tf",
            condition=IfCondition(use_wheel_odom_tf),
        ),

    ])
