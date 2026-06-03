import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    base_output_enable = LaunchConfiguration('base_output_enable')
    base_output_gravity_align = LaunchConfiguration('base_output_gravity_align')
    base_output_publish_tf = LaunchConfiguration('base_output_publish_tf')
    base_output_publish_odom = LaunchConfiguration('base_output_publish_odom')
    base_to_lio_x = LaunchConfiguration('base_to_lio_x')
    base_to_lio_y = LaunchConfiguration('base_to_lio_y')
    base_to_lio_z = LaunchConfiguration('base_to_lio_z')
    base_to_lio_roll = LaunchConfiguration('base_to_lio_roll')
    base_to_lio_pitch = LaunchConfiguration('base_to_lio_pitch')
    base_to_lio_yaw = LaunchConfiguration('base_to_lio_yaw')

    def optional_bool(context, launch_config):
        value = launch_config.perform(context).strip()
        if value == '':
            return None
        return value.lower() in ('true', '1', 'yes', 'on')

    def optional_float(context, launch_config):
        value = launch_config.perform(context).strip()
        if value == '':
            return None
        return float(value)

    def launch_setup(context, *args, **kwargs):
        parameter_files = [
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ]
        overrides = {}

        bool_overrides = (
            (base_output_enable, 'base_output.enable'),
            (base_output_gravity_align, 'base_output.gravity_align'),
            (base_output_publish_tf, 'base_output.publish_tf'),
            (base_output_publish_odom, 'base_output.publish_odom'),
        )
        for launch_config, parameter_name in bool_overrides:
            value = optional_bool(context, launch_config)
            if value is not None:
                overrides[parameter_name] = value

        trans = [
            optional_float(context, base_to_lio_x),
            optional_float(context, base_to_lio_y),
            optional_float(context, base_to_lio_z),
        ]
        if any(value is not None for value in trans):
            overrides['base_output.base_to_lio_trans'] = [value if value is not None else 0.0 for value in trans]

        rpy = [
            optional_float(context, base_to_lio_roll),
            optional_float(context, base_to_lio_pitch),
            optional_float(context, base_to_lio_yaw),
        ]
        if any(value is not None for value in rpy):
            overrides['base_output.base_to_lio_rpy'] = [value if value is not None else 0.0 for value in rpy]

        if overrides:
            parameter_files.append(overrides)

        fast_lio_node = Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=parameter_files,
            output='screen'
        )
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz_use)
        )

        return [fast_lio_node, rviz_node]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_base_output_enable_cmd = DeclareLaunchArgument(
        'base_output_enable', default_value='',
        description='Optional override for base_output.enable'
    )
    declare_base_output_gravity_align_cmd = DeclareLaunchArgument(
        'base_output_gravity_align', default_value='',
        description='Optional override for base_output.gravity_align'
    )
    declare_base_output_publish_tf_cmd = DeclareLaunchArgument(
        'base_output_publish_tf', default_value='',
        description='Optional override for base_output.publish_tf'
    )
    declare_base_output_publish_odom_cmd = DeclareLaunchArgument(
        'base_output_publish_odom', default_value='',
        description='Optional override for base_output.publish_odom'
    )
    declare_base_to_lio_x_cmd = DeclareLaunchArgument(
        'base_to_lio_x', default_value='',
        description='Optional override for base_output.base_to_lio_trans[0]'
    )
    declare_base_to_lio_y_cmd = DeclareLaunchArgument(
        'base_to_lio_y', default_value='',
        description='Optional override for base_output.base_to_lio_trans[1]'
    )
    declare_base_to_lio_z_cmd = DeclareLaunchArgument(
        'base_to_lio_z', default_value='',
        description='Optional override for base_output.base_to_lio_trans[2]'
    )
    declare_base_to_lio_roll_cmd = DeclareLaunchArgument(
        'base_to_lio_roll', default_value='',
        description='Optional override for base_output.base_to_lio_rpy[0]'
    )
    declare_base_to_lio_pitch_cmd = DeclareLaunchArgument(
        'base_to_lio_pitch', default_value='',
        description='Optional override for base_output.base_to_lio_rpy[1]'
    )
    declare_base_to_lio_yaw_cmd = DeclareLaunchArgument(
        'base_to_lio_yaw', default_value='',
        description='Optional override for base_output.base_to_lio_rpy[2]'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_base_output_enable_cmd)
    ld.add_action(declare_base_output_gravity_align_cmd)
    ld.add_action(declare_base_output_publish_tf_cmd)
    ld.add_action(declare_base_output_publish_odom_cmd)
    ld.add_action(declare_base_to_lio_x_cmd)
    ld.add_action(declare_base_to_lio_y_cmd)
    ld.add_action(declare_base_to_lio_z_cmd)
    ld.add_action(declare_base_to_lio_roll_cmd)
    ld.add_action(declare_base_to_lio_pitch_cmd)
    ld.add_action(declare_base_to_lio_yaw_cmd)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
