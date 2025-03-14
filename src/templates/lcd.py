#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# lcd dimensions
lcd_columns = 16
lcd_rows = 2

# I2C initialization
i2c = board.I2C()  # SCL = GPIO3 (pin 5), SDA = GPIO2 (pin 3)

# initiliazize class LCD
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.clear()

# set color bluee
lcd.color = [0, 100, 0]
time.sleep(1)
lcd.message = "Dagu Robot is..\nstarting now!"
time.sleep(3)


lcd.clear()

# change color to green
lcd.color = [0, 0, 100]
time.sleep(1)
lcd.message = "Seeking bottles\nLet's find them!"
time.sleep(3)

lcd.clear()

# change the color to red
lcd.color = [100, 0, 0]
time.sleep(1)
lcd.message = "The robot is\nshutdown for now!"
time.sleep(3)

# stop
lcd.color = [0, 0, 0]
lcd.clear()
