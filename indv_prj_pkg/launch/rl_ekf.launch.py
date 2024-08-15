from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Construct the path to the YAML configuration file
    config_file_path = os.path.join(
        get_package_share_directory('indv_prj_pkg'),
        'config',
        'ekf_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file_path],
        ),
    ])
