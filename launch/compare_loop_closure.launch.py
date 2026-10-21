import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    package_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    source_config = os.path.join(package_root, 'config', 'default.yaml')
    installed_config = os.path.join(package_path, 'config', 'default.yaml')
    default_config = source_config if os.path.exists(source_config) else installed_config

    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    common_remappings = [
        # Sensor inputs remain global and are therefore shared by both nodes.
        # Only estimator outputs are separated.
    ]

    def estimator(namespace, loop_enabled):
        prefix = '/' + namespace
        return Node(
            package='fast_lio',
            executable='fastlio_mapping',
            namespace=namespace,
            name='laser_mapping',
            parameters=[
                config_file,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'loop_closure.enable': loop_enabled,
                    'publish.corrected_map_topic': prefix + '/corrected_map',
                },
            ],
            remappings=common_remappings + [
                ('/Odometry', prefix + '/Odometry'),
                ('/uwfl2/navigation_health', prefix + '/navigation_health'),
                ('/uwfl2/sonar_diagnostics', prefix + '/sonar_diagnostics'),
                ('/uwfl2/loop_closure_diagnostics',
                 prefix + '/loop_closure_diagnostics'),
                ('/uwfl2_lc/raw_path', prefix + '/lc/raw_path'),
                ('/uwfl2_lc/optimized_path', prefix + '/lc/optimized_path'),
                ('/uwfl2_lc/markers', prefix + '/lc/markers'),
            ],
            output='screen',
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Absolute YAML configuration path used by both estimators'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the bag /clock'),
        estimator('lc_on', True),
        estimator('lc_off', False),
    ])
