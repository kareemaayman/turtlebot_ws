# PID Controller for TurtleBot

## Overview

This ROS node implements a PID (Proportional-Integral-Derivative) controller to autonomously navigate a TurtleBot to a specified target position and orientation. The controller processes odometry data to compute and send velocity commands that drive the robot towards the target.

### Key Components

- **PID Controller**: Adjusts the robot's linear and angular velocities based on errors in position and orientation.
- **Odometry Subscriber**: Receives real-time data about the robot's current position and orientation.
- **Velocity Publisher**: Sends computed velocity commands to the robot.

### How It Works

1. **Initialization**:
   - The ROS node initializes publishers and subscribers for handling velocity commands and odometry data.
   - PID parameters are set for both linear and angular velocities. These parameters determine how aggressively the robot responds to errors.

2. **Odometry Callback**:
   - The node subscribes to the `/odom` topic to get updates on the robot's position and orientation.
   - Converts the robot's orientation from quaternion to Euler angles to calculate the heading (theta).

3. **Control Computation**:
   - Computes the positional and angular errors between the robot's current state and the target.
   - Applies PID control to calculate the necessary linear and angular velocities.
   - Updates integral and derivative terms for the PID calculations.

4. **Publishing Commands**:
   - Sends computed velocity commands to the `/cmd_vel` topic to control the robot's movement.

5. **Stopping Condition**:
   - The robot stops when it is within a specified tolerance of the target position and orientation.

### Configuration

- **PID Gains**:
  - `Kp_lin`, `Ki_lin`, `Kd_lin`: Gains for the linear velocity PID controller.
  - `Kp_ang`, `Ki_ang`, `Kd_ang`: Gains for the angular velocity PID controller.

- **Target Values**:
  - `target_x`, `target_y`: Desired position coordinates.
  - `target_theta`: Desired orientation in radians.

- **Tolerance**:
  - `position_tolerance`: Acceptable deviation from the target position.
  - `angle_tolerance`: Acceptable deviation from the target orientation.
