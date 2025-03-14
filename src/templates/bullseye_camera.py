#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from picamera2 import Picamera2
from std_msgs.msg import Header

def main():
    rospy.init_node("picamera2_node", anonymous=True)
    pub = rospy.Publisher("camera/image", Image, queue_size=1)

    picam2 = Picamera2()
    # Configurăm explicit formatul "BGR888" ca să obținem 3 canale (BGR)
    config = picam2.create_video_configuration(
        main={
            "size": (640, 480),
            "format": "BGR888"
        }
    )
    picam2.configure(config)
    picam2.start()

    rate = rospy.Rate(30)  # 30fps
    while not rospy.is_shutdown():
        frame = picam2.capture_array()
        # Ar trebui să fie (480, 640, 3)
        rospy.loginfo_once(f"Frame shape: {frame.shape}")  # Debug la primul frame

        # Construim mesajul ROS
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = rospy.Time.now()
        img_msg.height, img_msg.width = frame.shape[:2]
        # Deoarece avem BGR888, encoding = "bgr8"
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.step = img_msg.width * 3
        img_msg.data = frame.tobytes()

        pub.publish(img_msg)
        rate.sleep()

    picam2.stop()

main()
