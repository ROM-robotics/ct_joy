from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_namespace',
            default_value='px4_1',
            description='Namespace for the PX4 controller node',
        ),
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
        ),
        Node(
            package='ct_joy',
            executable='ct_joy',
            namespace=LaunchConfiguration('px4_namespace'),
            name='f710_px4_controller',
            output='screen',
            parameters=[{
                'joy_topic': '/joy',
                'takeoff_altitude_m': 2.0,
                'max_horizontal_speed_mps': 3.0,
                'max_vertical_speed_mps': 1.0,
                'max_yaw_rate_rps': 1.5,
            }],
        ),
    ])
