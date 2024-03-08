import os

from ament_index_python.packages import get_package_share_directory

from launch import conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    sentry_decision_dir = get_package_share_directory('sentry_decision_v2')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = os.path.join(sentry_decision_dir, 'config', 'parameters.yaml')

    param_substitutions = {
        'use_sim_time': use_sim_time}

    configured_params = RewrittenYaml(
        source_file=config,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    sentry_decision_node = Node(
        package='sentry_decision_v2',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[configured_params],
    )

    ld = LaunchDescription([DeclareLaunchArgument('namespace', default_value='',
                            description='Top-level namespace'),
                            DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation (Gazebo) clock if true')])
    
    ld.add_action(sentry_decision_node)

    return ld