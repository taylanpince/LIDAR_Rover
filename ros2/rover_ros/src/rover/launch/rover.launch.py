import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_name = 'rover.urdf.xml'
    
    urdf = os.path.join(
        get_package_share_directory('rover'),
        urdf_file_name
    )

    print("Loading URDF from %s" % urdf)

    return LaunchDescription([
        Node(
            package='rover',
            namespace='rover_lidar',
            executable='lidar',
            name='rover_lidar'
        ),
        # Node(
        #     package='rover',
        #     executable='joint_states',
        #     name='joint_states',
        #     output='screen',
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False, 
                'robot_description': Command(['xacro ', urdf]),
            }],
        ),
        Node(
            name='tf2_ros_scan_link',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'scan_base', 'scan'],
        ),
        Node(
            name='tf2_ros_fp_map',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_link', 'map'],
        ),
        Node(
            name='tf2_ros_fp_odom',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_link', 'odom'],
        ),
        Node(
            name='tf2_ros_imu_link',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'imu_base', 'imu_link'],
        ),
    ])
