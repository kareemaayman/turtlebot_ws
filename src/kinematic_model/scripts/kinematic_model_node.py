#!/usr/bin/env python3
import numpy as np

# Vehicle parameters
L = 2.5  # Wheelbase (distance between front and rear axles) in meters
b = 0.2  # Distance between wheels (track width) in meters
dt = 0.1  # Time step in seconds
total_time = 10  # Total simulation time in seconds
r = 0.1  # Wheel radius in meters

# Wheel velocities (rad/sec)
wr_front = 10  # Angular velocity of the front right wheel
wl_front = 5  # Angular velocity of the front left wheel
wr_rear = 10  # Angular velocity of the rear right wheel
wl_rear = 5  # Angular velocity of the rear left wheel

# Convert angular velocities to linear velocities
vr_front = wr_front * r  # Linear velocity of the front right wheel
vl_front = wl_front * r  # Linear velocity of the front left wheel
vr_rear = wr_rear * r  # Linear velocity of the rear right wheel
vl_rear = wl_rear * r  # Linear velocity of the rear left wheel

# Calculate average linear velocity and yaw rate
Vx = (vr_front + vl_front + vr_rear + vl_rear) / 4  # Average forward velocity
Vy = 0  # For simplicity, assume no lateral movement
w = (vr_front - vl_front + vr_rear - vl_rear) / (4 * b)  # Average yaw rate

# Print the results
print(f"Vx = {Vx:.2f} m/s")
print(f"Vy = {Vy:.2f} m/s")
print(f"w = {w:.2f} rad/s")

# Initial position and orientation
x, y, theta = 0, 0, 0  # X position, Y position, orientation (yaw angle)

# Control inputs
v = Vx  # Linear velocity in m/s
delta_f = np.radians(10)  # Steering angle in radians

# Lists to store the trajectory
x_trajectory = []
y_trajectory = []

# Simulation loop
for _ in np.arange(0, total_time, dt):
    if delta_f != 0:
        R = L / np.tan(delta_f)  # Turning radius
        omega = v / R  # Angular velocity
    else:
        omega = 0

    # Update position and orientation
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt

    # Store the trajectory
    x_trajectory.append(x)
    y_trajectory.append(y)