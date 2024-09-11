#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller_node')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # PID parameters (tune these)
        self.Kp_lin = 0.5 # Proportional gain for linear velocity
        self.Ki_lin = 0.0 # Integral gain for linear velocity
        self.Kd_lin = 0.2  # Derivative gain for linear velocity

        self.Kp_ang = 0.4  # Proportional gain for angular velocity
        self.Ki_ang = 0.0  # Integral gain for angular velocity
        self.Kd_ang = 0.2  # Derivative gain for angular velocity

        self.target_x = 1.0  # target x position
        self.target_y = 1.0  # target y position
        self.target_theta = 0  # target orientation (radians)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.prev_pos_error = 0.0
        self.prev_angle_error = 0.0
        self.integral_pos_error = 0.0
        self.integral_angle_error = 0.0

        self.rate = rospy.Rate(10)  # Control loop frequency

        self.position_tolerance = 0.05  
        self.angle_tolerance = 0.05  

    def odom_callback(self, msg):
        # Update current position and orientation from Odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract quaternion orientation and convert to Euler angle (theta)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_theta) = euler_from_quaternion(orientation_list)

    def compute_control(self):
        # Positional error
        error_pos = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)

        # Angular error (ensure it's within -pi to pi)
        error_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x) - self.current_theta
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))  # Normalize angle to [-pi, pi]

        if abs(error_angle) < math.radians(10):  # Only move forward if the robot is facing the target
            self.integral_pos_error += error_pos
            derivative_pos_error = error_pos - self.prev_pos_error
            linear_velocity = self.Kp_lin * error_pos + self.Ki_lin * self.integral_pos_error + self.Kd_lin * derivative_pos_error
        else:
            linear_velocity = 0  # Stop linear movement while turning

        # PID control for angular velocity
        self.integral_angle_error += error_angle
        derivative_angle_error = error_angle - self.prev_angle_error
        angular_velocity = self.Kp_ang * error_angle + self.Ki_ang * self.integral_angle_error + self.Kd_ang * derivative_angle_error

        # Update previous errors
        self.prev_pos_error = error_pos
        self.prev_angle_error = error_angle

        # Publish velocity commands
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity
        self.cmd_pub.publish(cmd)

    def run(self):
        while not rospy.is_shutdown():
            # Stop if we are close enough to the goal
            if abs(self.target_x - self.current_x) < self.position_tolerance and \
               abs(self.target_y - self.current_y) < self.position_tolerance and \
               abs(self.target_theta - self.current_theta) < self.angle_tolerance:
                rospy.loginfo("Turtle bot reached, stopping movement")
                cmd = Twist()
                self.cmd_pub.publish(cmd)  # Stop the robot
                break

            # Compute and apply control commands
            self.compute_control()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = PIDController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
