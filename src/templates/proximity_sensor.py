#!/usr/bin/env python3
import sys
import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float32

TRIG = 23
ECHO1 = 24
ECHO2 = 5
ECHO3 = 27

def read_distance(echo_pin):
    GPIO.output(TRIG, False)
    time.sleep(0.001)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start_time = time.time()
    pulse_start = 0
    pulse_end = 0
    while GPIO.input(echo_pin) == 0:
        if time.time() - start_time > 0.05:
            return -1.0
        pulse_start = time.time()
    while GPIO.input(echo_pin) == 1:
        if time.time() - start_time > 0.05:
            return -1.0
        pulse_end = time.time()
    distance = (pulse_end - pulse_start) * 17150
    return round(distance, 2)

def proximity_sensors_publisher():
    try:
        rospy.init_node('proximity_sensors_node', anonymous=True)
    except rospy.ROSInitException:
        sys.exit(0)
    pub1 = rospy.Publisher('/sensor/proximity1', Float32, queue_size=10)
    pub2 = rospy.Publisher('/sensor/proximity2', Float32, queue_size=10)
    pub3 = rospy.Publisher('/sensor/proximity3', Float32, queue_size=10)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO1, GPIO.IN)
    GPIO.setup(ECHO2, GPIO.IN)
    GPIO.setup(ECHO3, GPIO.IN)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        dist1 = read_distance(ECHO1)
        dist2 = read_distance(ECHO2)
        dist3 = read_distance(ECHO3)
        rospy.loginfo(f"Dist1: {dist1}  Dist2: {dist2}  Dist3: {dist3}")
        pub1.publish(dist1)
        pub2.publish(dist2)
        pub3.publish(dist3)
        rate.sleep()
    GPIO.cleanup()

proximity_sensors_publisher()
