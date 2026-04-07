import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg_dir = get_package_share_directory("sim_test")
    loc_pkg_dir = get_package_share_directory("localization_layer")
    localization_launch = os.path.join(loc_pkg_dir, "launch", "cartographer_localization_launch.py")
    race_layer_launch = os.path.join(
        get_package_share_directory("race_layer"), "launch", "race_layer.launch.py"
    )
    default_params = os.path.join(sim_pkg_dir, "config", "sim_sensor_params_localization.yaml")

    pbstream_filename = LaunchConfiguration("pbstream_filename")
    world_type = LaunchConfiguration("world_type")
    map_yaml_path = LaunchConfiguration("map_yaml_path")
    centerline_csv_path = LaunchConfiguration("centerline_csv_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sensor_params_file = LaunchConfiguration("sensor_params_file")
    map_path_speed_mps = LaunchConfiguration("map_path_speed_mps")
    sim_motion_source = LaunchConfiguration("sim_motion_source")
    external_path_topic = LaunchConfiguration("external_path_topic")
    external_path_target_index = LaunchConfiguration("external_path_target_index")
    enable_race = LaunchConfiguration("enable_race")
    race_csv_path = LaunchConfiguration("race_csv_path")
    race_use_fgm = LaunchConfiguration("race_use_fgm")
    race_avoid_threshold = LaunchConfiguration("race_avoid_threshold")
    race_path_window_size = LaunchConfiguration("race_path_window_size")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pbstream_filename",
            default_value="/home/tkddn647/test/maps/sim_map_20260304_181103.pbstream",
            description="Absolute path to .pbstream map file used for pure localization",
        ),
        DeclareLaunchArgument("world_type", default_value="map"),
        DeclareLaunchArgument(
            "map_yaml_path",
            default_value="/home/tkddn647/test/maps/sim_map_20260304_181103_rosmap.yaml",
        ),
        # Do not use auto-extracted polar path in localization runs.
        # It can deviate at tight corners and destabilize scan matching.
        DeclareLaunchArgument(
            "centerline_csv_path",
            default_value="/home/tkddn647/test/maps/sim_map_20260304_181103_centerline.csv",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("sensor_params_file", default_value=default_params),
        # Keep localization motion conservative. High speed with sparse scan causes
        # scan-matching jumps in repetitive track sections.
        DeclareLaunchArgument("map_path_speed_mps", default_value="0.25"),
        DeclareLaunchArgument(
            "sim_motion_source",
            default_value="local_path",
            description="sim motion source: internal_path or local_path",
        ),
        DeclareLaunchArgument(
            "external_path_topic",
            default_value="/recommended_path",
            description="Path topic used when sim_motion_source=local_path (default: stable recommended path)",
        ),
        DeclareLaunchArgument(
            "external_path_target_index",
            default_value="2",
            description="Forward target index in external path follower",
        ),
        DeclareLaunchArgument("enable_race", default_value="true"),
        DeclareLaunchArgument(
            "race_csv_path",
            default_value="/home/tkddn647/test/maps/sim_map_20260304_181103_centerline.csv",
            description="Race-layer path CSV (default: centerline)",
        ),
        DeclareLaunchArgument("race_use_fgm", default_value="false"),
        DeclareLaunchArgument("race_avoid_threshold", default_value="0.7"),
        # Use full path by default to avoid TF-unavailable startup causing
        # index-0 window lock and trajectory pull to a wrong segment.
        DeclareLaunchArgument("race_path_window_size", default_value="0"),
        Node(
            package="tf_manager_cpp",
            executable="sensor_static_tf",
            name="sensor_static_tf_node",
            output="screen",
        ),
        Node(
            package="sim_test",
            executable="sim_fake_sensor_publisher.py",
            name="sim_fake_sensor_publisher",
            output="screen",
            parameters=[
                sensor_params_file,
                {
                    "world_type": world_type,
                    "map_yaml_path": map_yaml_path,
                    "centerline_csv_path": centerline_csv_path,
                    "map_path_speed_mps": map_path_speed_mps,
                    "motion_source": sim_motion_source,
                    "external_path_topic": external_path_topic,
                    "external_path_target_index": external_path_target_index,
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "pbstream_filename": pbstream_filename,
                "use_sim_time": use_sim_time,
                "imu_topic": "/ebimu/imu",
                "odom_topic": "/odom",
                "scan_topic": "/scan",
                "enable_sensor_bringup": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(race_layer_launch),
            condition=IfCondition(enable_race),
            launch_arguments={
                "csv_path": race_csv_path,
                "path_window_size": race_path_window_size,
                "avoid_threshold": race_avoid_threshold,
                "use_fgm": race_use_fgm,
                "map_frame": "map",
                "base_frame": "base_link",
                "laser_frame": "laser",
            }.items(),
        ),
    ])
