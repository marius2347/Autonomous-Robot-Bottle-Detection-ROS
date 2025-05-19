#!/usr/bin/env python3
"""
cmd_vel_subscriber.py (modified)
—————————
Subscribes to /cmd_vel and three proximity sensors,
executes: stop → pause → backup → side-turn → drive forward on front obstacle.
"""

import sys, time, os, random, rospy, pyfirmata2, serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

########################  USER-TUNABLE CONSTANTS  ########################
L                    = 0.5    # wheel baseline (m)
ARDUINO_PORT         = '/dev/ttyACM0' if os.path.exists('/dev/ttyACM0') else '/dev/ttyACM1'

obstacle_threshold   = 40.0   # cm – front sensor threshold
avoid_speed          = 0.3    # PWM duty (0-1) for avoidance maneuvers
backup_distance      = 0.3    # m – reverse distance during avoidance
TURN_SEC             = 5.0    # turn duration (s)
STOP_SEC             = 5.0    # pause duration when obstacle detected
#########################################################################

#                    GLOBAL STATE (updated in callbacks)                #
last_front      = float('inf')
last_right      = float('inf')
last_left       = float('inf')
obstacle_active = False
#########################################################################

############################  ARDUINO / MOTORS  #########################

def reset_arduino(port):
    ser = serial.Serial(port, 9600)
    ser.dtr = False; time.sleep(1); ser.dtr = True; ser.close()


def set_wheel(pwm, d1, d2, speed):
    d1.write(int(speed > 0)); d2.write(int(speed < 0))
    pwm.write(min(abs(speed), 1.0))


def stop():
    set_wheel(M1_PWM, M1_D1, M1_D2, 0)
    set_wheel(M2_PWM, M2_D1, M2_D2, 0)


def reverse(sec):
    rospy.loginfo("REVERSING for %.2f s", sec)
    set_wheel(M1_PWM, M1_D1, M1_D2, -avoid_speed)
    set_wheel(M2_PWM, M2_D1, M2_D2, -avoid_speed)
    time.sleep(sec)
    stop()


def turn_left(sec=TURN_SEC):
    rospy.loginfo("TURNING LEFT for %.2f s", sec)
    set_wheel(M1_PWM, M1_D1, M1_D2, -avoid_speed)
    set_wheel(M2_PWM, M2_D1, M2_D2,  avoid_speed)
    time.sleep(sec)
    stop()


def turn_right(sec=TURN_SEC):
    rospy.loginfo("TURNING RIGHT for %.2f s", sec)
    set_wheel(M1_PWM, M1_D1, M1_D2,  avoid_speed)
    set_wheel(M2_PWM, M2_D1, M2_D2, -avoid_speed)
    time.sleep(sec)
    stop()


def drive_forward():
    rospy.loginfo("DRIVING FORWARD at avoid_speed")
    set_wheel(M1_PWM, M1_D1, M1_D2, avoid_speed)
    set_wheel(M2_PWM, M2_D1, M2_D2, avoid_speed)

############################  AVOIDANCE SEQUENCE  ########################
def perform_avoidance():
    global obstacle_active
    obstacle_active = True

    rospy.loginfo("---- Avoidance sequence START ----")
    rospy.loginfo("Stopping for %.2f s", STOP_SEC)
    stop(); time.sleep(STOP_SEC)

    backup_sec = backup_distance / avoid_speed
    rospy.loginfo("Backing up %.2f m (%.2f s)", backup_distance, backup_sec)
    reverse(backup_sec)

    rospy.loginfo("Checking sides: left=%.2f cm, right=%.2f cm", last_left, last_right)
    if last_left > obstacle_threshold:
        rospy.loginfo("Left side clear (%.2f cm) → turning left", last_left)
        turn_left()
    elif last_right > obstacle_threshold:
        rospy.loginfo("Right side clear (%.2f cm) → turning right", last_right)
        turn_right()
    else:
        choice = random.choice(["left", "right"])
        rospy.loginfo("Neither side clear → random %s turn", choice)
        turn_left() if choice == "left" else turn_right()

    drive_forward()
    rospy.loginfo("Avoidance sequence COMPLETE")
    obstacle_active = False

############################  ROS CALLBACKS  ###########################
def prox_front_cb(msg):
    global last_front
    last_front = msg.data
    rospy.loginfo("Front sensor: %.2f cm", last_front)
    if not obstacle_active and last_front < obstacle_threshold:
        perform_avoidance()


def prox_right_cb(msg):
    global last_right
    last_right = msg.data
    rospy.loginfo("Right sensor: %.2f cm", last_right)


def prox_left_cb(msg):
    global last_left
    last_left = msg.data
    rospy.loginfo("Left sensor: %.2f cm", last_left)

############################  ROS BOILERPLATE  ##########################
if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_motor_control')

    reset_arduino(ARDUINO_PORT)
    board = pyfirmata2.Arduino(ARDUINO_PORT)
    M1_PWM = board.get_pin('d:9:p');  M1_D1 = board.get_pin('d:2:o'); M1_D2 = board.get_pin('d:4:o')
    M2_PWM = board.get_pin('d:10:p'); M2_D1 = board.get_pin('d:7:o'); M2_D2 = board.get_pin('d:8:o')
    board.samplingOn()

    rospy.Subscriber('/sensor/proximity1', Float32, prox_front_cb)
    rospy.Subscriber('/sensor/proximity2', Float32, prox_right_cb)
    rospy.Subscriber('/sensor/proximity3', Float32, prox_left_cb)

    rospy.loginfo("Node ready – threshold: %.2f cm, backup: %.2f m", obstacle_threshold, backup_distance)
    rospy.spin()
