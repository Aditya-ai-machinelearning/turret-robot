from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='turret_camera',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),

        Node(
            package='turret_vision',
            executable='detection_node',
            name='detection_node',
            output='screen'
        ),

        Node(
            package='turret_vision',
            executable='tracker_node',
            name='tracker_node',
            output='screen'
        ),

        Node(
            package='turret_control',
            executable='pid_node',
            name='pid_node',
            output='screen'
        ),

        Node(
            package='turret_control',
            executable='sc15_node',
            name='sc15_node',
            output='screen'
        ),

        Node(
            package='turret_control',
            executable='fire_node',
            name='fire_node',
            output='screen'
        ),

        Node(
            package='turret_control',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        Node(
            package='turret_drive',
            executable='drive_node',
            name='drive_node',
            output='screen'
        ),

        Node(
            package='turret_control',
            executable='dashboard_node',
            name='dashboard_node',
            output='screen'
        ),

    ])