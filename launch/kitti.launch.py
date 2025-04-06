from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    fast_lio_dir = get_package_share_directory('fast_lio_localization_sc_qn')
    config_path = os.path.join(fast_lio_dir, 'config', 'kitti.yaml')  
    
    with open(config_path, 'r') as file:
        configParams = yaml.safe_load(file)['kitti']['ros__parameters']   
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description=''
        ),

        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[
                configParams,
                {
                    'feature_extract_enable': False,
                    'point_filter_num': 4,
                    'max_iteration': 3,
                    'filter_size_surf': 0.5,
                    'filter_size_map': 0.5,
                    'cube_side_length': 1000.0,
                    'runtime_pos_log_enable': False
                }
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('fast_lio_localization_sc_qn'),
                'rviz',
                'loam_livox.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': False}]
        )
    ])
