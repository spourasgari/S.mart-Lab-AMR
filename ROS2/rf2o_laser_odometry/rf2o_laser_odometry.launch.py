import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o', # Change it to /odom when you want to use the odometry only from this source (recommended, not necessary)
                    'publish_tf' : True, # Set to true when you want to use the odometry only from this source (Don't forget to also publish base_footprint -> base_link and base_link -> laser)
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0}],
            ),
    ])
