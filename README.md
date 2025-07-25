# ROS2 Path Smoothing and Trajectory Control

# Objective
This project implements a path smoothing algorithm and a trajectory tracking controller for a simulated differential drive robot (Turtlebot3) navigating through a series of 2D waypoints. The goal is to generate a smooth, time-parameterized trajectory from discrete waypoints and ensure the robot follows this trajectory accurately, with an added feature for reactive obstacle avoidance.

# Features

- ```Path Smoothing```: Transforms a set of discrete 2D waypoints into a smooth, continuous path using cubic spline interpolation.

- ```Trajectory Generation```: Creates a time-parameterized trajectory from the smoothed path, assigning timestamps to each point based on a constant desired linear velocity.
      
- ```Trajectory Tracking Controller```: Implements a proportional (P) controller to guide a simulated differential drive robot (Turtlebot3) along the generated trajectory.
      
- ```Reactive Obstacle Avoidance```: Integrates a basic reactive behavior using laser scan data to detect and avoid immediate obstacles by turning or stopping.

- ```ROS2 Integration```: All functionalities are implemented as modular ROS2 nodes, communicating via standard ROS2 messages.
      
- ```Simulation & Visualization```: Demonstrates robot movement and path following in Gazebo and Rviz2.

# Environment Setup
- Ubuntu version - 22.04
- ROS2 version – Humble
      
# Prerequisites
Ensure the following packages and dependencies are installed before setting up the workspace.

- ROS2
- Python 3.8+
- Required Python Packages:

      pip install numpy scipy transforms3d
      
- Turtlebot3 Simulation Packages:

      sudo apt update
      
      sudo apt install ros-humble-turtlebot3-simulations ros-humble-gazebo-ros-pkgs
      
      (Replace humble with your ROS2 distribution if different).
      
# Setup Instructions

- Workspace and Package Creation

	    mkdir -p ~/ros2_ws/src
	
	    cd ~/ros2_ws/src
	

# Building the Package

	cd ~/ros2_ws
	
	colcon build
	
	source install/setup.bash
	
# Execution Instructions


- Launching the Simulation

1. set turtlebot3 model:

       export turtlebot3_model=burger 
       
2. launch the combined simulation:

       ros2 launch smooth_path_controller full_simulation.launch.py
       

3. Visualizing with Rviz2

	    rviz2
	
4. Main controller:

	    ros2 launch smooth_path_controller final_launch.py
	

# Design Choices & Algorithms

# ROS2 Architecture

The solution is designed with modularity in mind, leveraging ROS2's distributed architecture:
- raw_waypoints_publisher: A simple node responsible for publishing a predefined set of discrete 2D waypoints as a geometry_msgs/PoseArray. 

- path_smoother: Subscribes to /raw_waypoints, applies a smoothing algorithm, and publishes the smoothed_path as another geometry_msgs/PoseArray.

- trajectory_generator: Subscribes to /smoothed_path, time-parameterizes it, and publishes the trajectory as a nav_msgs/Path (a list of geometry_msgs/PoseStamped messages with timestamps).

- trajectory_tracker: The main control node. It subscribes to the trajectory from the generator, the robot's odom (odometry) for its current pose, and scan for obstacle detection. It then publishes geometry_msgs/Twist commands to cmd_vel to drive the robot.


**1. Path Smoothing**
- Algorithm: Cubic Spline Interpolation (specifically, using scipy.interpolate.CubicSpline).

- Rationale: Cubic splines are chosen because they produce a smooth, continuous curve that passes directly through all the given waypoints. This ensures that the robot's path is physically traversable and avoids sharp turns, which can be problematic for real robots. The scipy implementation provides a robust and mathematically sound method for this.


**2. Trajectory Generation**
- Algorithm: Constant Velocity Time Parameterization.

- Rationale: From the smoothed path, a time-stamped trajectory is generated. Each point on the smoothed path is assigned a timestamp based on the cumulative distance from the start and a predefined desired_linear_velocity. This creates a time-based reference for the robot to follow, allowing the controller to know where the robot should be at any given time.


**3. Trajectory Tracking Controller**
- Algorithm: Proportional (P) Controller with Look-Ahead.

- Rationale: A P-controller is a simple yet effective control strategy for trajectory following. It calculates control commands (linear and angular velocities) proportional to the error between the robot's current state and its desired state.


**4. Obstacle Avoidance** 
- Algorithm: Reactive Obstacle Avoidance (Simple Frontal Check & Turn/Stop).

- Rationale: This feature provides a basic level of safety by enabling the robot to react to unforeseen obstacles not accounted for in the global path. It prioritizes collision avoidance over strict trajectory following when an immediate threat is detected.

# Extensibility to a Real Robot

- Extending this solution to a real robot would involve several key considerations:

**1. Hardware Interface:**

- Odometry: A real robot would provide odometry data (position and orientation) via wheel encoders, IMUs, or a combination. The odom topic would be published by the robot's low-level drivers.
- Laser Scanner: A physical LiDAR or depth camera would publish sensor_msgs/LaserScan or sensor_msgs/PointCloud2 data.

- Motor Control: The geometry_msgs/Twist commands published by the trajectory_tracker would need to be translated into specific motor commands (e.g., PWM signals for left/right wheel velocities) by the robot's base controller. This involves inverse kinematics for the differential drive model.

**2. Localization:**

- Odometry alone drifts over time. A real robot would require a robust localization system (e.g., AMCL - Adaptive Monte Carlo Localization, or an Extended Kalman Filter/Particle Filter) that fuses odometry with sensor data (like LiDAR scans against a map) to provide accurate global pose estimates. This would typically publish to the /tf tree and potentially a more accurate /odom or /map frame.

**3. Mapping:**

- For navigation in unknown environments, a Simultaneous Localization and Mapping (SLAM) system (e.g., Cartographer, Gmapping, Karto) would be needed to build a map of the environment.

4. ROS2 Navigation Stack (Nav2):

- For production-grade autonomous navigation, this project's components would typically be integrated into or replaced by the ROS2 Navigation Stack (Nav2). Nav2 provides:

- Global Planner: Generates a path on a costmap.

- Local Planner: Generates velocity commands for obstacle avoidance and path following  using DWA or TEB.

- Costmaps: Representations of the environment with obstacle information.

- State Estimators: For robust localization.

- Path smoothing and trajectory generation could serve as a custom global path refinement layer before a local planner takes over. The controller could be a custom local planner.



# AI Usage

- Conceptualization: Brainstorming algorithms for path smoothing, trajectory generation, and control.

- Code Generation: Generating initial boilerplate code for ROS2 nodes and logic for splines.

- Debugging: Identifying and troubleshooting various runtime errors, import issues, and logical flaws (e.g., AttributeError, AssertionError, time synchronization problems).


# Future Improvements

- Dynamic Velocity Profiles: Implement trapezoidal or S-curve velocity profiles for smoother acceleration/deceleration along the trajectory, rather than constant velocity.

- Advanced Controllers: Explore more sophisticated controllers like PID (Proportional-Integral-Derivative) for better error reduction, or Model Predictive Control (MPC) for optimal control considering robot dynamics and constraints.

- Global Obstacle Avoidance: Integrate with a global planner that can plan paths around known static obstacles using a map (e.g., by providing waypoints that already avoid obstacles).

- Dynamic Obstacle Handling: Implement more advanced local planning algorithms (e.g., Dynamic Window Approach (DWA), Timed Elastic Band (TEB), ORCA) that can predict and avoid moving obstacles.

- Waypoint Input GUI: Create a simple Rviz2 plugin or a web-based GUI to allow users to click and define waypoints dynamically, rather than hardcoding them.

- Error Handling and Robustness: Add more comprehensive error handling (e.g., for invalid scan data, unreachable target points, or odometry loss).

