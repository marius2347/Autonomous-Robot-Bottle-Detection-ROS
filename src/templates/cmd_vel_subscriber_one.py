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

L = 0.5
port = '/dev/ttyACM0'
if not os.path.exists(port):
    port = '/dev/ttyACM1'

obstacle_threshold = 50.0
avoid_speed = 0.2
user_linear_x = 0.0
user_angular_z = 0.0
obstacle_detected = False
last_distance = float('inf')

def reset_arduino(port):
    ser = serial.Serial(port, 9600)
    ser.dtr = False
    time.sleep(1)
    ser.dtr = True
    ser.close()

def set_wheel(M_PWM, M_DIR1, M_DIR2, speed):
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
    left = user_linear_x - (user_angular_z * L / 2.0)
    right = user_linear_x + (user_angular_z * L / 2.0)
    set_wheel(M1_PWM, M1_DIR1, M1_DIR2, left)
    set_wheel(M2_PWM, M2_DIR1, M2_DIR2, right)

def turn_left(duration=1.0):
    set_wheel(M1_PWM, M1_DIR1, M1_DIR2, -avoid_speed)
    set_wheel(M2_PWM, M2_DIR1, M2_DIR2, avoid_speed)
    time.sleep(duration)
    stop()

def turn_right(duration=1.0):
    set_wheel(M1_PWM, M1_DIR1, M1_DIR2, avoid_speed)
    set_wheel(M2_PWM, M2_DIR1, M2_DIR2, -avoid_speed)
    time.sleep(duration)
    stop()

def avoid_obstacle():
    global obstacle_detected
    obstacle_detected = True
    stop()
    rospy.loginfo("Avoiding obstacle: %.2f cm", last_distance)
    if random.choice([True, False]):
        rospy.loginfo("Turning LEFT")
        turn_left()
    else:
        rospy.loginfo("Turning RIGHT")
        turn_right()
    drive_user_speed()

def cmd_vel_callback(msg):
    global obstacle_detected, user_linear_x, user_angular_z
    user_linear_x = msg.linear.x
    user_angular_z = msg.angular.z
    obstacle_detected = False
    drive_user_speed()

def proximity_callback(msg):
    global last_distance, obstacle_detected
    last_distance = msg.data
    rospy.loginfo("Proximity: %.2f cm", last_distance)
    if last_distance < obstacle_threshold and not obstacle_detected:
        avoid_obstacle()

def on_shutdown():
    rospy.loginfo("Shutting down, stopping motors.")
    stop()
    M1_PWM.write(0)
    M1_DIR1.write(0)
    M1_DIR2.write(0)
    M2_PWM.write(0)
    M2_DIR1.write(0)
    M2_DIR2.write(0)
    time.sleep(0.5)
    board.exit()

try:
    rospy.init_node('cmd_vel_to_motor_control')
except rospy.ROSInitException:
    print("ROS not running. Exiting...")
    sys.exit(0)

reset_arduino(port)
board = pyfirmata2.Arduino(port)
M1_PWM = board.get_pin('d:9:p')
M1_DIR1 = board.get_pin('d:2:o')
M1_DIR2 = board.get_pin('d:4:o')
M2_PWM = board.get_pin('d:10:p')
M2_DIR1 = board.get_pin('d:7:o')
M2_DIR2 = board.get_pin('d:8:o')
board.samplingOn()
rospy.on_shutdown(on_shutdown)
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
rospy.Subscriber('/sensor/proximity', Float32, proximity_callback)
rospy.loginfo("Node started. Waiting for cmd_vel.")
rospy.spin()
