#!/usr/bin/env python3
import sys
import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float32

GPIO.setmode(GPIO.BCM)
TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def read_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.01)  
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

def proximity_sensor_publisher():
    try:
        rospy.init_node('proximity_sensor_node', anonymous=True)
    except rospy.ROSInitException:
        print("ROS not started. Exiting...")
        sys.exit(0)

    pub = rospy.Publisher('/sensor/proximity', Float32, queue_size=10)

    
    rate = rospy.Rate(20)

    rospy.loginfo("Proximity sensor node started...")

    try:
        while not rospy.is_shutdown():
            distance = read_distance()
            rospy.loginfo(f"Measured distance: {distance} cm")
            pub.publish(distance)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Cleaning up GPIO...")
        GPIO.cleanup()

proximity_sensor_publisher()
