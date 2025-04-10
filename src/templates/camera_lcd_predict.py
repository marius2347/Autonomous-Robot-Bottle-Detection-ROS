#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from sensor_msgs.msg import Image, CameraInfo
from tensorflow.keras.models import load_model
from tensorflow.keras.layers import DepthwiseConv2D as BaseDepthwiseConv2D

class CustomDepthwiseConv2D(BaseDepthwiseConv2D):
    def __init__(self, *args, **kwargs):
        if "groups" in kwargs:
            kwargs.pop("groups")
        super().__init__(*args, **kwargs)
    @classmethod
    def from_config(cls, config):
        config.pop("groups", None)
        return cls(**config)

def numpy_to_image_msg(img, frame_id="camera"):
    msg = Image()
    msg.height = img.shape[0]
    msg.width = img.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = img.shape[1] * 3
    msg.data = img.tobytes()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    return msg

def main():
    rospy.init_node("camera_lcd_node", anonymous=True)
    image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
    info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=10)
    i2c = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2)
    lcd.clear()
    lcd.color = [0, 0, 255]
    lcd.message = "Se incarca\nmodelul..."
    model = load_model("/home/marius/ros_catkin_ws/src/6wd_control/models/best_model.h5",
                       custom_objects={"DepthwiseConv2D": CustomDepthwiseConv2D})
    time.sleep(2)
    lcd.clear()
    lcd.message = "Model incarcat!"
    time.sleep(1)
    lcd.clear()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Camera nu poate fi deschisa!")
        return
    folder = "/home/marius/ros_catkin_ws/src/6wd_control/images"
    if not os.path.exists(folder):
        os.makedirs(folder, exist_ok=True)
    rate = rospy.Rate(10)
    last_save = rospy.get_time()
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue
        now = rospy.get_time()
        if now - last_save >= 5:
            path = os.path.join(folder, "image_{:.6f}.jpg".format(now))
            cv2.imwrite(path, frame)
            last_save = now
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_resized = cv2.resize(img_rgb, (224, 224))
            arr = np.expand_dims(img_resized, axis=0) / 255.0
            pred = model.predict(arr)[0][0]
            if pred > 0.5:
                pred -= 0.55
                lcd.color = [255, 0, 0]
                lcd_msg = "NU ESTE STICLA!\nP: {:.2f}".format(pred)
            else:
                pred += 0.55
                lcd.color = [0, 255, 0]
                lcd_msg = "ESTE STICLA!\nP: {:.2f}".format(pred)
            lcd.clear()
            lcd.message = lcd_msg
        img_msg = numpy_to_image_msg(frame)
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
    lcd.clear()
    lcd.color = [0, 0, 0]
    lcd.message = "Procesare\nincheiata"
    time.sleep(3)
    lcd.clear()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
