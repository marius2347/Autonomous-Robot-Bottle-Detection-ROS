#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image, CameraInfo

def numpy_to_image_msg(np_img, frame_id="camera"):
    msg = Image()
    msg.height = np_img.shape[0]
    msg.width = np_img.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = np_img.shape[1] * 3
    msg.data = np_img.tobytes()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    return msg

def camera_publisher():
    rospy.init_node('camera_node', anonymous=True)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Camera nu poate fi deschisă!")
        return

    image_folder = "/home/marius/ros_catkin_ws/src/6wd_control/images"
    if not os.path.exists(image_folder):
        os.makedirs(image_folder, exist_ok=True)

    rate = rospy.Rate(10)
    last_save_time = rospy.get_time()
    rospy.loginfo("Nodul de cameră a pornit...")
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Nu s-a capturat o imagine!")
            continue

        current_time = rospy.get_time()
        if current_time - last_save_time >= 5:
            filename = os.path.join(image_folder, "image_{:.6f}.jpg".format(current_time))
            cv2.imwrite(filename, frame)
            last_save_time = current_time

        img_msg = numpy_to_image_msg(frame, frame_id="camera")
        image_pub.publish(img_msg)

        ci = CameraInfo()
        ci.header.stamp = rospy.Time.now()
        ci.header.frame_id = "camera"
        ci.width = frame.shape[1]
        ci.height = frame.shape[0]
        fx = frame.shape[1] / 2.0
        fy = frame.shape[0] / 2.0
        cx = frame.shape[1] / 2.0
        cy = frame.shape[0] / 2.0
        ci.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        ci.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        ci.D = [0, 0, 0, 0, 0]
        ci.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        info_pub.publish(ci)

        rate.sleep()
    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
