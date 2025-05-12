from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_map_server_launch_file_dir = os.path.join(get_package_share_directory('nav2_map_server'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_map_server_launch_file_dir, '/map_saver_server.launch.py']),
        ),
        Node(
            package='map_server_extension', 
            executable='map_mgmt_server', 
            name='map_mgmt_server_extension',
            parameters=[{'use_sim_time':use_sim_time}]
        )
    ])