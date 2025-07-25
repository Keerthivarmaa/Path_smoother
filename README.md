# Path_smoother

ROS2 Path Smoothing and Trajectory Control in 2D Space
Objective
This project implements a path smoothing algorithm and a trajectory tracking controller for a simulated differential drive robot (Turtlebot3) navigating through a series of 2D waypoints. The goal is to generate a smooth, time-parameterized trajectory from discrete waypoints and ensure the robot follows this trajectory accurately, with an added feature for reactive obstacle avoidance.

Table of Contents
Features

Project Structure

Setup Instructions

Prerequisites

Workspace and Package Creation

Code Placement

Updating setup.py

Creating the Launch File

Building the Package

Execution Instructions

Terminating Previous Runs (Important!)

Launching the Simulation

Visualizing with Rviz2

Adding Obstacles in Gazebo

Design Choices & Algorithms

ROS2 Architecture

1. Path Smoothing

2. Trajectory Generation

3. Trajectory Tracking Controller

4. Obstacle Avoidance (Extra Credit)

Extensibility to a Real Robot

AI Tools Used

Future Improvements

Features
Path Smoothing: Transforms a set of discrete 2D waypoints into a smooth, continuous path using cubic spline interpolation.

Trajectory Generation: Creates a time-parameterized trajectory from the smoothed path, assigning timestamps to each point based on a constant desired linear velocity.

Trajectory Tracking Controller: Implements a proportional (P) controller to guide a simulated differential drive robot (Turtlebot3) along the generated trajectory.

Reactive Obstacle Avoidance (Extra Credit): Integrates a basic reactive behavior using laser scan data to detect and avoid immediate obstacles by turning or stopping.

ROS2 Integration: All functionalities are implemented as modular ROS2 nodes, communicating via standard ROS2 messages.

Simulation & Visualization: Demonstrates robot movement and path following in Gazebo and Rviz2.

Project Structure
ros2_ws/
├── src/
│   └── smooth_path_controller/
│       ├── smooth_path_controller/
│       │   ├── __init__.py
│       │   ├── raw_waypoints_publisher.py
│       │   ├── path_smoother.py
│       │   ├── trajectory_generator.py
│       │   └── trajectory_tracker.py
│       ├── launch/
│       │   └── full_simulation.launch.py
│       └── setup.py
│       └── package.xml
└── install/
└── build/
└── log/

Setup Instructions
Prerequisites
Ubuntu 20.04+ (or other Linux distribution supported by ROS2)

ROS2 Humble Hawksbill (or a compatible ROS2 distribution) installed and sourced.

Verify with ros2 --version.

Python 3.8+

Required Python Packages:

pip install numpy scipy transforms3d

Turtlebot3 Simulation Packages:

sudo apt update
sudo apt install ros-humble-turtlebot3-simulations ros-humble-gazebo-ros-pkgs

(Replace humble with your ROS2 distribution if different).

Workspace and Package Creation
If you don't have an existing ROS2 workspace, create one:

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python smooth_path_controller

Code Placement
Navigate into the newly created package directory:

cd ~/ros2_ws/src/smooth_path_controller
mkdir smooth_path_controller # Create a directory inside the package with the same name

Place your Python scripts in the ~/ros2_ws/src/smooth_path_controller/smooth_path_controller/ directory:

raw_waypoints_publisher.py

path_smoother.py

trajectory_generator.py

trajectory_tracker.py

Updating setup.py
Open ~/ros2_ws/src/smooth_path_controller/setup.py and ensure the entry_points and data_files sections are correctly configured to expose your nodes as executables and include the launch file.

from setuptools import find_packages, setup

package_name = 'smooth_path_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/full_simulation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # Replace with your name
    maintainer_email='your_email@example.com', # Replace with your email
    description='ROS2 Path Smoothing and Trajectory Control',
    license='Apache-2.0', # Or your chosen license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'raw_waypoints_publisher = smooth_path_controller.raw_waypoints_publisher:main',
            'path_smoother = smooth_path_controller.path_smoother:main',
            'trajectory_generator = smooth_path_controller.trajectory_generator:main',
            'trajectory_tracker = smooth_path_controller.trajectory_tracker:main',
        ],
    },
)

Creating the Launch File
Create a launch directory inside your package:

mkdir ~/ros2_ws/src/smooth_path_controller/launch

Save the following content as ~/ros2_ws/src/smooth_path_controller/launch/full_simulation.launch.py. This launch file starts Gazebo, spawns the Turtlebot3, and then launches all your path planning and tracking nodes with a delay to ensure Gazebo is ready.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get the share directory of the turtlebot3_gazebo package
    turtlebot3_gazebo_share_dir = get_package_share_directory('turtlebot3_gazebo')
    
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
    # Obstacle avoidance parameters
    obstacle_min_distance_arg = DeclareLaunchArgument(
        'obstacle_min_distance',
        default_value='0.4',
        description='Minimum distance to an obstacle to react (meters)'
    )
    obstacle_detection_angle_range_arg = DeclareLaunchArgument(
        'obstacle_detection_angle_range',
        default_value='0.5', # 0.5 rad = ~28 degrees
        description='Angle range (radians) in front to check for obstacles'
    )
    avoidance_angular_speed_arg = DeclareLaunchArgument(
        'avoidance_angular_speed',
        default_value='0.5',
        description='Angular speed when avoiding (rad/s)'
    )
    avoidance_linear_speed_reduction_arg = DeclareLaunchArgument(
        'avoidance_linear_speed_reduction',
        default_value='0.2',
        description='Factor to reduce linear speed when avoiding'
    )


    # Launch Gazebo world
    gazebo_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'model': 'burger'}.items() # Ensure model is passed
    )

    # Define the spawn_entity node (as it's usually done within turtlebot3_world.launch.py)
    # We need to explicitly define it here to add a delay
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-file', os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models', 'turtlebot3_burger', 'model.sdf'
            ),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '--ros-args'
        ],
        output='screen',
        emulate_tty=True,
    )

    # Define your path planning nodes
    path_planning_nodes = [
        Node(
            package='smooth_path_controller',
            executable='raw_waypoints_publisher',
            name='raw_waypoints_publisher',
            output='screen',
            emulate_tty=True,
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
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'desired_linear_velocity': LaunchConfiguration('desired_linear_velocity_gen')
            }]
        ),
        Node(
            package='smooth_path_controller',
            executable='trajectory_tracker',
            name='trajectory_tracker',
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
                'obstacle_min_distance': LaunchConfiguration('obstacle_min_distance'),
                'obstacle_detection_angle_range': LaunchConfiguration('obstacle_detection_angle_range'),
                'avoidance_angular_speed': LaunchConfiguration('avoidance_angular_speed'),
                'avoidance_linear_speed_reduction': LaunchConfiguration('avoidance_linear_speed_reduction'),
            }]
        ),
    ]

    return LaunchDescription([
        kp_linear_arg,
        kp_angular_arg,
        look_ahead_time_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        goal_tolerance_distance_arg,
        goal_tolerance_angle_arg,
        desired_linear_velocity_gen_arg,
        obstacle_min_distance_arg,
        obstacle_detection_angle_range_arg,
        avoidance_angular_speed_arg,
        avoidance_linear_speed_reduction_arg,

        # 1. Launch Gazebo World
        gazebo_world_launch,

        # 2. Add a TimerAction to delay spawning the robot and launching your nodes
        TimerAction(
            period=7.0,  # Adjust this delay (in seconds) as needed. 5-10 seconds is usually good.
            actions=[
                spawn_robot_node,
                *path_planning_nodes # Unpack your list of nodes
            ]
        )
    ])

Building the Package
Go back to your workspace root and build your package. This compiles the Python scripts and makes them executable.

cd ~/ros2_ws
colcon build --packages-select smooth_path_controller

After building, source your workspace in every new terminal you open for ROS2 commands:

source install/setup.bash

Execution Instructions
Terminating Previous Runs (Important!)
Before launching, ensure no old Gazebo or ROS2 processes are running. This is crucial for a clean start.

Stop any active ros2 launch processes: Go to the terminal where you ran ros2 launch and press Ctrl+C. Wait for processes to terminate.

Verify ROS2 nodes are stopped:

ros2 node list

If you still see nodes like /gazebo, /turtlebot3_diff_drive, etc., they are stuck.

Find their PIDs: ros2 node info <node_name> (look for Pid: <number>).

Kill them: kill <PID> (or kill -9 <PID> if kill fails).

Verify Gazebo processes are stopped:

ps aux | grep gzserver
ps aux | grep gzclient

If you see any, kill them: kill -9 <PID_of_gzserver> and kill -9 <PID_of_gzclient>.

Alternatively: killall -9 gzserver gzclient (use with caution).

Last resort: ros2 daemon stop

Launching the Simulation
Open a new terminal (and remember to source install/setup.bash):

Set Turtlebot3 Model:

export TURTLEBOT3_MODEL=burger # Or waffle_pi

Launch the combined simulation:

ros2 launch smooth_path_controller full_simulation.launch.py

This will open the Gazebo simulator window and start all your ROS2 nodes. Observe the terminal for [INFO] messages from each node. There will be a 7.0 second delay before the robot spawns and your nodes start processing.

Visualizing with Rviz2
Open another new terminal (and source install/setup.bash):

rviz2

In the Rviz2 window:

Global Options (Left Panel):

Set Fixed Frame to odom.

Add Displays (Left Panel, "Add" button):

Click "Add" (bottom left).

Go to the By Topic tab and add:

/raw_waypoints (Type: PoseArray)

/smoothed_path (Type: PoseArray)

/trajectory (Type: Path)

/odom (Type: Odometry)

/scan (Type: LaserScan) - Crucial for visualizing obstacle detection.

Go to the By Display Type tab and add:

RobotModel (ensure Description Topic is /robot_description)

TF

Observe:

You should see the raw waypoints, the smooth path, and the trajectory in Rviz2.

The Turtlebot3 model will appear and start moving, following the trajectory.

The /scan topic will show laser beams, indicating obstacles.

Experiment with the launch file parameters (kp_linear, kp_angular, look_ahead_time, desired_linear_velocity_gen) to fine-tune tracking performance.

Adding Obstacles in Gazebo
To test the obstacle avoidance feature:

In the Gazebo GUI window:

Look for the "Insert" tab on the left panel (it might be collapsed).

Drag and drop simple models like "Cylinder" or "Box" into the world.

Place them directly in the robot's path to observe its reactive avoidance behavior.

Observe in Rviz2/Gazebo:

The /scan topic in Rviz2 will show laser hits on the obstacle.

The robot should slow down, turn, or stop depending on the obstacle_min_distance and the obstacle's position relative to the robot's front.

You can adjust obstacle_min_distance, obstacle_detection_angle_range, avoidance_angular_speed, and avoidance_linear_speed_reduction in full_simulation.launch.py to modify avoidance behavior.

Design Choices & Algorithms
ROS2 Architecture
The solution is designed with modularity in mind, leveraging ROS2's distributed architecture:

raw_waypoints_publisher: A simple node responsible for publishing a predefined set of discrete 2D waypoints as a geometry_msgs/PoseArray. This simulates the output of a global planner.

path_smoother: Subscribes to /raw_waypoints, applies a smoothing algorithm, and publishes the smoothed_path as another geometry_msgs/PoseArray.

trajectory_generator: Subscribes to /smoothed_path, time-parameterizes it, and publishes the trajectory as a nav_msgs/Path (a list of geometry_msgs/PoseStamped messages with timestamps).

trajectory_tracker: The main control node. It subscribes to the trajectory from the generator, the robot's odom (odometry) for its current pose, and scan for obstacle detection. It then publishes geometry_msgs/Twist commands to cmd_vel to drive the robot.

This modular design allows for independent development, testing, and easy replacement of individual components (e.g., swapping out the smoothing algorithm or controller).

1. Path Smoothing
Algorithm: Cubic Spline Interpolation (specifically, using scipy.interpolate.CubicSpline).

Rationale: Cubic splines are chosen because they produce a smooth, continuous curve that passes directly through all the given waypoints. This ensures that the robot's path is physically traversable and avoids sharp turns, which can be problematic for real robots. The scipy implementation provides a robust and mathematically sound method for this.

Implementation: The path_smoother.py node takes the raw waypoints, creates cubic spline interpolators for X and Y coordinates independently over a normalized time parameter, and then samples these splines at a higher resolution to generate the dense smoothed path.

2. Trajectory Generation
Algorithm: Constant Velocity Time Parameterization.

Rationale: From the smoothed path, a time-stamped trajectory is generated. Each point on the smoothed path is assigned a timestamp based on the cumulative distance from the start and a predefined desired_linear_velocity. This creates a time-based reference for the robot to follow, allowing the controller to know where the robot should be at any given time.

Implementation: The trajectory_generator.py node calculates the distance between consecutive points on the smoothed path. It then sums these distances to get a cumulative distance and divides by the desired_linear_velocity to obtain the time increment for each segment, assigning an absolute timestamp to each pose in the nav_msgs/Path message.

3. Trajectory Tracking Controller
Algorithm: Proportional (P) Controller with Look-Ahead.

Rationale: A P-controller is a simple yet effective control strategy for trajectory following. It calculates control commands (linear and angular velocities) proportional to the error between the robot's current state and its desired state.

Time-Based Target Selection: Instead of simply targeting the next waypoint, the controller finds a target_pose on the trajectory that corresponds to the current_time + look_ahead_time. This "look-ahead" mechanism helps the robot anticipate turns and smooth out its motion, preventing it from constantly lagging behind or overshooting.

Error Calculation:

Distance Error: Euclidean distance from the robot's current position to the target_pose. This drives the linear velocity.

Heading Error: Angular difference between the robot's current orientation and the bearing to the target_pose. This drives the angular velocity. The error is normalized to [-pi, pi] to ensure the shortest turn direction.

Clamping: Velocity commands are clamped to max_linear_velocity and max_angular_velocity to ensure safe and realistic robot motion.

Goal Reaching: The controller includes a check to stop the robot when it is within a goal_tolerance_distance of the final trajectory point.

Implementation: The trajectory_tracker.py node continuously calculates these errors and outputs geometry_msgs/Twist messages. ROS parameters are used for kp_linear, kp_angular, look_ahead_time, and velocity limits, allowing for easy tuning without recompilation.

4. Obstacle Avoidance (Extra Credit)
Algorithm: Reactive Obstacle Avoidance (Simple Frontal Check & Turn).

Rationale: This feature provides a basic level of safety by enabling the robot to react to unforeseen obstacles not accounted for in the global path. It prioritizes collision avoidance over strict trajectory following when an immediate threat is detected.

Implementation:

The trajectory_tracker.py node subscribes to sensor_msgs/LaserScan messages (from the simulated laser scanner on the Turtlebot3).

A check_for_obstacles() method is implemented. This method examines laser scan readings within a defined obstacle_detection_angle_range (a sector in front of the robot).

If any obstacle is detected within obstacle_min_distance, the controller's behavior is overridden:

If the obstacle is very close, the robot stops.

Otherwise, it reduces its linear speed and commands an angular velocity to turn away from the obstacle (e.g., turn right if the obstacle is more to the left, and vice-versa).

The track_trajectory function checks for obstacles first. If an obstacle is detected, it executes the avoidance behavior and skips the regular trajectory tracking logic for that control cycle.

Limitations: This is a purely reactive approach. It does not re-plan the path and might get stuck in complex environments (e.g., U-shaped obstacles).

Extensibility to a Real Robot
Extending this solution to a real robot would involve several key considerations:

Hardware Interface:

Odometry: A real robot would provide odometry data (position and orientation) via wheel encoders, IMUs, or a combination. The odom topic would be published by the robot's low-level drivers.

Laser Scanner: A physical LiDAR or depth camera would publish sensor_msgs/LaserScan or sensor_msgs/PointCloud2 data.

Motor Control: The geometry_msgs/Twist commands published by the trajectory_tracker would need to be translated into specific motor commands (e.g., PWM signals for left/right wheel velocities) by the robot's base controller. This involves inverse kinematics for the differential drive model.

Localization:

Odometry alone drifts over time. A real robot would require a robust localization system (e.g., AMCL - Adaptive Monte Carlo Localization, or an Extended Kalman Filter/Particle Filter) that fuses odometry with sensor data (like LiDAR scans against a map) to provide accurate global pose estimates. This would typically publish to the /tf tree and potentially a more accurate /odom or /map frame.

Mapping:

For navigation in unknown environments, a Simultaneous Localization and Mapping (SLAM) system (e.g., Cartographer, Gmapping, Karto) would be needed to build a map of the environment.

ROS2 Navigation Stack (Nav2):

For production-grade autonomous navigation, this project's components would typically be integrated into or replaced by the ROS2 Navigation Stack (Nav2). Nav2 provides:

Global Planner: Generates a path (similar to your raw_waypoints_publisher) on a costmap.

Local Planner: Generates velocity commands for obstacle avoidance and path following (more sophisticated than your reactive approach, often using DWA or TEB).

Costmaps: Representations of the environment with obstacle information.

State Estimators: For robust localization.

Your path smoothing and trajectory generation could serve as a custom global path refinement layer before a local planner takes over. Your controller could be a custom local planner.

Parameter Tuning: The kp_linear, kp_angular, and avoidance parameters would need extensive real-world tuning on the physical robot, as simulation dynamics are never perfectly accurate.

Safety: Real robots require additional safety layers (e.g., emergency stop buttons, hardware-level collision detection) that are beyond software control.

AI Tools Used
This project's development was actively supported by an AI-powered large language model (LLM), specifically Gemini. The LLM assisted with:

Conceptualization: Brainstorming algorithms for path smoothing, trajectory generation, and control.

Code Generation: Generating initial boilerplate code for ROS2 nodes, Python logic for splines, kinematics, and controller.

Debugging: Identifying and troubleshooting various runtime errors, import issues, and logical flaws (e.g., AttributeError, AssertionError, time synchronization problems, and the "robot going straight" issue).

Documentation: Structuring and drafting this comprehensive README.md file, including explanations of algorithms and extensibility.

The iterative feedback loop with the LLM significantly accelerated the development and debugging process, allowing for rapid prototyping and problem-solving.

Future Improvements
Dynamic Velocity Profiles: Implement trapezoidal or S-curve velocity profiles for smoother acceleration/deceleration along the trajectory, rather than constant velocity.

Advanced Controllers: Explore more sophisticated controllers like PID (Proportional-Integral-Derivative) for better error reduction, or Model Predictive Control (MPC) for optimal control considering robot dynamics and constraints.

Global Obstacle Avoidance: Integrate with a global planner that can plan paths around known static obstacles using a map (e.g., by providing waypoints that already avoid obstacles).

Dynamic Obstacle Handling: Implement more advanced local planning algorithms (e.g., Dynamic Window Approach (DWA), Timed Elastic Band (TEB), ORCA) that can predict and avoid moving obstacles.

Waypoint Input GUI: Create a simple Rviz2 plugin or a web-based GUI to allow users to click and define waypoints dynamically, rather than hardcoding them.

Error Handling and Robustness: Add more comprehensive error handling (e.g., for invalid scan data, unreachable target points, or odometry loss).

Parameter Server Integration: Use ROS2 parameter server for dynamic parameter tuning at runtime without restarting nodes.