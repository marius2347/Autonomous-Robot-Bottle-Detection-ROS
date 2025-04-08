#!/usr/bin/env python3
import rospy
import time
import os
import cv2
import numpy as np
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
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

def main():
    rospy.init_node("lcd_display_node", anonymous=True)
    
    lcd_columns = 16
    lcd_rows = 2
    i2c = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.clear()

    lcd.color = [0, 0, 255]  # albastru
    lcd.message = "Se incarca\nmodelul..."
    
    best_model = load_model('/home/marius/ros_catkin_ws/src/6wd_control/models/best_model.h5',
                            custom_objects={'DepthwiseConv2D': CustomDepthwiseConv2D})
    time.sleep(2)
    lcd.clear()
    lcd.message = "Model incarcat!"
    time.sleep(1)
    lcd.clear()
    
    image_folder = "/home/marius/ros_catkin_ws/src/6wd_control/images"
    image_files = [f for f in os.listdir(image_folder) if f.lower().endswith((".jpg", ".png"))]
    if not image_files:
        lcd.message = "Nu sunt imagini!"
        rospy.logwarn("Nu sunt imagini in folderul %s", image_folder)
        return

    image_files.sort()

    for image_file in image_files:
        image_path = os.path.join(image_folder, image_file)
        img = cv2.imread(image_path)
        if img is None:
            rospy.logwarn("Nu se poate citi imaginea: %s", image_path)
            continue

        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (224, 224))
        img_array = np.expand_dims(img_resized, axis=0) / 255.0

        prediction = best_model.predict(img_array)[0][0]
        if prediction > 0.5:
            lcd_msg = "STICLA ESTE\nNEDETECTATA\nP: {:.2f}".format(prediction)
            lcd.color = [255, 0, 0]  # roșu pentru ne-detectare
        else:
            lcd_msg = "STICLA ESTE\nDETECTATA\nP: {:.2f}".format(prediction)
            lcd.color = [0, 255, 0]  # verde pentru detectare

        lcd.clear()
        lcd.message = lcd_msg
        rospy.loginfo("Imagine: %s -> %s", image_file, lcd_msg)
        
        time.sleep(3)
    
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
