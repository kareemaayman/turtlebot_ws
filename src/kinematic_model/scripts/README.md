# 4WD Vehicle Kinematic Model Simulation

This Python script simulates the trajectory of a 4-wheel drive (4WD) vehicle using a simple kinematic model. The model computes the vehicle's motion based on its wheel velocities, steering angle, and other vehicle parameters.

## Features

- **Kinematic Model**: The script calculates the linear velocity, yaw rate, and updates the vehicle's position and orientation over time.
- **Trajectory Plotting**: The trajectory of the vehicle is plotted on a 2D plane, allowing visualization of the vehicle's path over the simulation period.
- **Customizable Parameters**: Vehicle parameters such as wheelbase, track width, wheel angular velocities, and steering angle can be easily modified.

## Vehicle Parameters

- **Wheelbase**: Distance between the front and rear axles (default: `2.5` meters).
- **Track Width**: Distance between the left and right wheels (default: `0.2` meters).
- **Wheel Radius**: Radius of each wheel (default: `0.1` meters).
- **Time Step**: Time increment for each simulation step (default: `0.1` seconds).
- **Total Simulation Time**: Total time for the simulation (default: `10` seconds).

## Control Inputs

- **Wheel Angular Velocities**: 
  - Right Front Wheel: `10` radians/second
  - Left Front Wheel: `5` radians/second
  - Right Rear Wheel: `10` radians/second
  - Left Rear Wheel: `5` radians/second
- **Steering Angle**: The angle of the steering in radians (default: `10` degrees converted to radians).

## How the Simulation Works

1. **Linear Velocity Calculation**:
   - The angular velocities of the wheels are converted into linear velocities using the wheel radius.
   - The average linear velocity of the vehicle is computed as the mean of the linear velocities of all wheels.

2. **Yaw Rate Calculation**:
   - The yaw rate, which represents the rate of change of the vehicle's orientation, is calculated based on the difference between the velocities of the wheels on opposite sides.

3. **Position and Orientation Update**:
   - The position (X, Y) and orientation (theta) of the vehicle are updated iteratively using the linear velocity, yaw rate, and steering angle.

## Customization

You can modify the vehicle parameters and control inputs directly in the script to simulate different vehicle behaviors. Adjust the angular velocities, steering angle, and vehicle dimensions as needed.
