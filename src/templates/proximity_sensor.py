#!/usr/bin/env python3
import sys
import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float32

# ────────────── PIN ASSIGNMENTS ──────────────
TRIG1, ECHO1 = 25, 23   # front
TRIG2, ECHO2 = 16,  5   # right
TRIG3, ECHO3 = 26, 27   # left

# list helpers for compact loops
TRIG_PINS = [TRIG1, TRIG2, TRIG3]
ECHO_PINS = [ECHO1, ECHO2, ECHO3]

def read_distance(trig_pin, echo_pin):
    """Trigger one sensor and compute distance in cm (-1.0 on timeout)."""
    GPIO.output(trig_pin, False)
    time.sleep(0.0002)                          # 200 µs settle
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)                         # 10 µs pulse
    GPIO.output(trig_pin, False)

    start_time = time.time()
    pulse_start = pulse_end = 0

    # wait for rising edge
    while GPIO.input(echo_pin) == 0:
        if time.time() - start_time > 0.06:     # 60 ms timeout
            return -1.0
        pulse_start = time.time()

    # wait for falling edge
    while GPIO.input(echo_pin) == 1:
        if time.time() - start_time > 0.06:
            return -1.0
        pulse_end = time.time()

    distance = (pulse_end - pulse_start) * 17150.0
    return round(distance, 2)

def proximity_sensors_publisher():
    try:
        rospy.init_node("proximity_sensors_node", anonymous=True)
    except rospy.ROSInitException:
        sys.exit(0)

    pubs = [
        rospy.Publisher("/sensor/proximity1", Float32, queue_size=10),
        rospy.Publisher("/sensor/proximity2", Float32, queue_size=10),
        rospy.Publisher("/sensor/proximity3", Float32, queue_size=10),
    ]

    GPIO.setmode(GPIO.BCM)
    # configure all TRIG pins output LOW, all ECHO pins input
    for trig in TRIG_PINS:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.output(trig, False)
    for echo in ECHO_PINS:
        GPIO.setup(echo, GPIO.IN)

    rate = rospy.Rate(20)      # 20 Hz loop (~6.7 Hz per sensor)
    last_log = time.time()

    while not rospy.is_shutdown():
        distances = [
            read_distance(TRIG1, ECHO1),   # front
            read_distance(TRIG2, ECHO2),   # right
            read_distance(TRIG3, ECHO3)    # left
        ]

        # log once per second
        now = time.time()
        if now - last_log >= 1.0:
            rospy.loginfo(
                "Front: %.2f cm   Right: %.2f cm   Left: %.2f cm",
                *distances
            )
            last_log = now

        for pub, d in zip(pubs, distances):
            pub.publish(d)

        rate.sleep()

    GPIO.cleanup()

if __name__ == "__main__":
    proximity_sensors_publisher()
