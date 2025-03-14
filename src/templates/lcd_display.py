#!/usr/bin/env python3
import rospy
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

def main():
    rospy.init_node("lcd_display_node", anonymous=True)
    lcd_columns = 16
    lcd_rows = 2
    i2c = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.clear()
    lcd.color = [0, 100, 0]
    time.sleep(1)
    lcd.message = "Dagu Robot is..\nstarting now!"
    time.sleep(3)
    lcd.clear()
    lcd.color = [0, 0, 100]
    time.sleep(1)
    lcd.message = "Seeking bottles\nLet's find them!"
    time.sleep(3)
    lcd.clear()
    lcd.color = [100, 0, 0]
    time.sleep(1)
    lcd.message = "The robot is\nshutdown for now!"
    time.sleep(3)
    lcd.color = [0, 0, 0]
    lcd.clear()

main()
