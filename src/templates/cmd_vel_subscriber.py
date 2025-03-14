#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
import pyfirmata2
import serial
import time
import random
from std_msgs.msg import Float32
import os

# Distance between wheels, for differential drive calculation
L = 0.5

# Try to auto-detect the Arduino port
port = '/dev/ttyACM0'
if not os.path.exists(port):
    port = '/dev/ttyACM1'

# Obstacle avoidance settings
obstacle_threshold = 50.0
avoid_speed = 0.2

# Global variables for user commands
user_linear_x = 0.0
user_angular_z = 0.0

# We now have three distances to track
last_distance1 = float('inf')
last_distance2 = float('inf')
last_distance3 = float('inf')

# Keep track if we’re currently avoiding an obstacle
obstacle_detected = False

############################################
# Arduino reset and motor control
############################################
def reset_arduino(port):
    ser = serial.Serial(port, 9600)
    ser.dtr = False
    time.sleep(1)
    ser.dtr = True
    ser.close()

def set_wheel(M_PWM, M_DIR1, M_DIR2, speed):
    """
    speed > 0 => forward
    speed < 0 => backward
    speed = 0 => stop
    """
    if speed > 0:
        M_DIR1.write(1)
        M_DIR2.write(0)
        M_PWM.write(min(abs(speed), 1.0))
    elif speed < 0:
        M_DIR1.write(0)
        M_DIR2.write(1)
        M_PWM.write(min(abs(speed), 1.0))
    else:
        M_DIR1.write(0)
        M_DIR2.write(0)
        M_PWM.write(0)

def stop():
    set_wheel(M1_PWM, M1_DIR1, M1_DIR2, 0)
    set_wheel(M2_PWM, M2_DIR1, M2_DIR2, 0)

def drive_user_speed():
    """
    Simple differential drive:
    left_wheel_speed  = linear_x - (angular_z * L/2)
    right_wheel_speed = linear_x + (angular_z * L/2)
    """
    left = user_linear_x - (user_angular_z * L / 2.0)
    right = user_linear_x + (user_angular_z * L / 2.0)
    set_wheel(M1_PWM, M1_DIR1, M1_DIR2, left)
    set_wheel(M2_PWM, M2_DIR1, M2_DIR2, right)

def turn_left(duration=1.0):
    set_wheel(M1_PWM, M1_DIR1, M1_DIR2, -avoid_speed)
    set_wheel(M2_PWM, M2_DIR1, M2_DIR2,  avoid_speed)
    time.sleep(duration)
    stop()

def turn_right(duration=1.0):
    set_wheel(M1_PWM, M1_DIR1, M1_DIR2,  avoid_speed)
    set_wheel(M2_PWM, M2_DIR1, M2_DIR2, -avoid_speed)
    time.sleep(duration)
    stop()

def avoid_obstacle():
    """
    Stop, then randomly turn left or right, then resume user speed.
    """
    global obstacle_detected
    obstacle_detected = True  # Let everyone know we’re in "avoidance" mode
    stop()
    rospy.loginfo("Avoiding obstacle...")
    
    # Random choice to turn left or right
    if random.choice([True, False]):
        rospy.loginfo("Turning LEFT")
        turn_left()
    else:
        rospy.loginfo("Turning RIGHT")
        turn_right()
    
    # Resume user command after avoid
    drive_user_speed()

############################################
# ROS Callbacks
############################################
def cmd_vel_callback(msg):
    """
    Stores the latest linear/angular velocity command
    and tries to drive at that speed unless avoiding obstacle.
    """
    global user_linear_x, user_angular_z, obstacle_detected
    user_linear_x = msg.linear.x
    user_angular_z = msg.angular.z

    # If we are not in obstacle avoidance, drive user speed
    obstacle_detected = False
    drive_user_speed()

def check_all_sensors():
    """
    Checks if any sensor is below the threshold; if so, calls avoid_obstacle().
    """
    global last_distance1, last_distance2, last_distance3, obstacle_detected

    # If any distance < obstacle_threshold => avoid
    if (
        last_distance1 < obstacle_threshold or
        last_distance2 < obstacle_threshold or
        last_distance3 < obstacle_threshold
    ) and not obstacle_detected:
        avoid_obstacle()

def proximity1_callback(msg):
    """
    Callback for sensor 1.
    Ignores invalid readings (i.e., -1 or any negative).
    """
    global last_distance1
    if msg.data < 0:
        # Don't update or log negative/invalid readings
        return

    last_distance1 = msg.data
    rospy.loginfo("Proximity1: %.2f cm", last_distance1)
    check_all_sensors()

def proximity2_callback(msg):
    """
    Callback for sensor 2.
    Ignores invalid readings (i.e., -1 or any negative).
    """
    global last_distance2
    if msg.data < 0:
        return

    last_distance2 = msg.data
    rospy.loginfo("Proximity2: %.2f cm", last_distance2)
    check_all_sensors()

def proximity3_callback(msg):
    """
    Callback for sensor 3.
    Ignores invalid readings (i.e., -1 or any negative).
    """
    global last_distance3
    if msg.data < 0:
        return

    last_distance3 = msg.data
    rospy.loginfo("Proximity3: %.2f cm", last_distance3)
    check_all_sensors()

def on_shutdown():
    rospy.loginfo("Shutting down, stopping motors.")
    stop()
    # Ensure PWM and DIR pins are off
    M1_PWM.write(0)
    M1_DIR1.write(0)
    M1_DIR2.write(0)
    M2_PWM.write(0)
    M2_DIR1.write(0)
    M2_DIR2.write(0)
    time.sleep(0.5)
    board.exit()

############################################
# Main Node Init
############################################
try:
    rospy.init_node('cmd_vel_to_motor_control')
except rospy.ROSInitException:
    print("ROS not running. Exiting...")
    sys.exit(0)

# Setup the Arduino
reset_arduino(port)
board = pyfirmata2.Arduino(port)

# Setup the pins
M1_PWM   = board.get_pin('d:9:p')
M1_DIR1  = board.get_pin('d:2:o')
M1_DIR2  = board.get_pin('d:4:o')
M2_PWM   = board.get_pin('d:10:p')
M2_DIR1  = board.get_pin('d:7:o')
M2_DIR2  = board.get_pin('d:8:o')

board.samplingOn()

# Register shutdown hook
rospy.on_shutdown(on_shutdown)

# Subscribers
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
rospy.Subscriber('/sensor/proximity1', Float32, proximity1_callback)
rospy.Subscriber('/sensor/proximity2', Float32, proximity2_callback)
rospy.Subscriber('/sensor/proximity3', Float32, proximity3_callback)

rospy.loginfo("cmd_vel_to_motor_control node started. Waiting for /cmd_vel and sensor topics.")
rospy.spin()
