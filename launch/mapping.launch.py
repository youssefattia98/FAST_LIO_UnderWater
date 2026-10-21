import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    package_root_from_launch = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    source_default_config = os.path.join(package_root_from_launch, 'config', 'default.yaml')
    installed_default_config = os.path.join(package_path, 'config', 'default.yaml')
    default_config_file = source_default_config if os.path.exists(source_default_config) else installed_default_config
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    loop_closure_enable = LaunchConfiguration('loop_closure_enable')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value=default_config_file,
        description='Absolute yaml config file path'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_loop_closure_enable_cmd = DeclareLaunchArgument(
        'loop_closure_enable', default_value='auto',
        description=(
            "Loop-closure override: 'true', 'false', or 'auto' to use the YAML")
    )

    def launch_nodes(context):
        parameter_overrides = {
            'use_sim_time': use_sim_time.perform(context).lower() in
                            ('true', '1', 'yes', 'on')
        }
        loop_override = loop_closure_enable.perform(context).lower()
        if loop_override not in ('auto', 'true', 'false'):
            raise RuntimeError(
                "loop_closure_enable must be 'auto', 'true', or 'false'")
        if loop_override != 'auto':
            parameter_overrides['loop_closure.enable'] = loop_override == 'true'

        return [
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                parameters=[config_file.perform(context), parameter_overrides],
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_cfg],
                condition=IfCondition(rviz_use)
            )
        ]

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_loop_closure_enable_cmd)

    ld.add_action(OpaqueFunction(function=launch_nodes))

    return ld
