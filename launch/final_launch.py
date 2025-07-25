import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for controller parameters
    kp_linear_arg = DeclareLaunchArgument(
        'kp_linear',
        default_value='0.8',
        description='Proportional gain for linear velocity in trajectory tracker'
    )
    kp_angular_arg = DeclareLaunchArgument(
        'kp_angular',
        default_value='1.5',
        description='Proportional gain for angular velocity in trajectory tracker'
    )
    look_ahead_time_arg = DeclareLaunchArgument(
        'look_ahead_time',
        default_value='0.5',
        description='Look ahead time in seconds for trajectory tracker'
    )
    max_linear_velocity_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='0.5',
        description='Maximum linear velocity for the robot'
    )
    max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='1.0',
        description='Maximum angular velocity for the robot'
    )
    goal_tolerance_distance_arg = DeclareLaunchArgument(
        'goal_tolerance_distance',
        default_value='0.1',
        description='Distance tolerance to consider goal reached'
    )
    goal_tolerance_angle_arg = DeclareLaunchArgument(
        'goal_tolerance_angle',
        default_value='0.1',
        description='Angle tolerance to consider goal orientation reached'
    )
    desired_linear_velocity_gen_arg = DeclareLaunchArgument(
        'desired_linear_velocity_gen',
        default_value='0.5',
        description='Desired linear velocity for trajectory generation'
    )


    return LaunchDescription([
        kp_linear_arg,
        kp_angular_arg,
        look_ahead_time_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        goal_tolerance_distance_arg,
        goal_tolerance_angle_arg,
        desired_linear_velocity_gen_arg,

        Node(
            package='smooth_path_controller',
            executable='waypoints',
            name='waypoints',
            output='screen',
            emulate_tty=True, # For colored output in terminal
        ),
        Node(
            package='smooth_path_controller',
            executable='path_smoother',
            name='path_smoother',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='smooth_path_controller',
            executable='trajectory',
            name='trajectory',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'desired_linear_velocity': LaunchConfiguration('desired_linear_velocity_gen')
            }]
        ),
        Node(
            package='smooth_path_controller',
            executable='controller',
            name='controller',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'kp_linear': LaunchConfiguration('kp_linear'),
                'kp_angular': LaunchConfiguration('kp_angular'),
                'look_ahead_time': LaunchConfiguration('look_ahead_time'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'goal_tolerance_distance': LaunchConfiguration('goal_tolerance_distance'),
                'goal_tolerance_angle': LaunchConfiguration('goal_tolerance_angle'),
            }]
        ),
    ])
