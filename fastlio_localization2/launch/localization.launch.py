from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio_localization")
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    legacy_map_path = LaunchConfiguration("map")
    pcd_map_path_arg = LaunchConfiguration("pcd_map_path")
    pcd_map_path = PythonExpression([
        "'",
        pcd_map_path_arg,
        "' if '",
        pcd_map_path_arg,
        "' else '",
        legacy_map_path,
        "'",
    ])
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    initialpose_topic = LaunchConfiguration("initialpose_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )
    declare_rviz_cmd = DeclareLaunchArgument("rviz", default_value="true", description="Use RViz to monitor results")

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg", default_value=default_rviz_config_path, description="RViz config file path"
    )

    declare_map_path = DeclareLaunchArgument("map", default_value="", description="Path to PCD map file")
    declare_pcd_map_path = DeclareLaunchArgument("pcd_map_path", default_value="", description="Path to horizontal PCD map file")
    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic", default_value="/map", description="Topic to publish PCD map"
    )
    declare_scan_topic = DeclareLaunchArgument(
        "scan_topic", default_value="/cloud_registered_aligned", description="External FAST-LIO aligned scan topic"
    )
    declare_odom_topic = DeclareLaunchArgument(
        "odom_topic", default_value="/nav_odom", description="External FAST-LIO odometry topic"
    )
    declare_initialpose_topic = DeclareLaunchArgument(
        "initialpose_topic", default_value="/initialpose", description="Initial map->base_link pose topic"
    )
    declare_map_frame = DeclareLaunchArgument("map_frame", default_value="map", description="Map frame")
    declare_odom_frame = DeclareLaunchArgument("odom_frame", default_value="odom", description="Odometry frame")
    declare_base_frame = DeclareLaunchArgument("base_frame", default_value="base_link", description="Robot base frame")

    # Global localization node
    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization.py",
        name="global_localization",
        output="screen",
        parameters=[{"map_voxel_size": 0.4,
                     "scan_voxel_size": 0.1,
                     "freq_localization": 0.5,
                     "freq_global_map": 0.25,
                     "localization_threshold": 0.9,
                     "fov": 6.28319,
                     "fov_far": 300,
                     "pcd_map_path": pcd_map_path,
                     "pcd_map_topic": pcd_map_topic,
                     "scan_topic": scan_topic,
                     "odom_topic": odom_topic,
                     "initialpose_topic": initialpose_topic,
                     "map_frame": map_frame,
                     "odom_frame": odom_frame,
                     "base_frame": base_frame}],
    )

    # Transform fusion node
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
        parameters=[{"odom_topic": odom_topic,
                     "map_to_odom_topic": "/map_to_odom",
                     "map_frame": map_frame,
                     "odom_frame": odom_frame,
                     "base_frame": base_frame,
                     "publish_tf": True}],
    )
    
    # PCD to PointCloud2 publisher
    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{"file_name": pcd_map_path,
                     "tf_frame": map_frame,
                    "cloud_topic": pcd_map_topic,
                    "period_ms_": 500}],
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ]
    )

    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], condition=IfCondition(rviz_use))

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_path)
    ld.add_action(declare_pcd_map_path)
    ld.add_action(declare_pcd_map_topic)
    ld.add_action(declare_scan_topic)
    ld.add_action(declare_odom_topic)
    ld.add_action(declare_initialpose_topic)
    ld.add_action(declare_map_frame)
    ld.add_action(declare_odom_frame)
    ld.add_action(declare_base_frame)

    ld.add_action(rviz_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(pcd_publisher_node)

    return ld
