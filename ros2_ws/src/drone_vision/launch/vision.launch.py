from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = LaunchConfiguration('config')
    upright = ParameterValue(LaunchConfiguration('camera_upright_confirmed'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(
            get_package_share_directory('drone_vision'), 'config', 'vision.yaml')),
        DeclareLaunchArgument('camera_upright_confirmed', default_value='false'),
        # Restart only vision processes; launch shutdown suppresses respawn. No flight commands.
        Node(package='drone_vision', executable='camera_node', name='camera_node', parameters=[config], output='screen', respawn=True, respawn_delay=2.0),
        Node(package='drone_vision', executable='tag_detector_node', name='tag_detector_node', parameters=[config], output='screen', respawn=True, respawn_delay=2.0),
        Node(package='drone_vision', executable='follow_preview_node', name='follow_preview_node',
             parameters=[config, {'camera_upright_confirmed': upright}], output='screen', respawn=True, respawn_delay=2.0),
    ])
