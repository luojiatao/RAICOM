#!/usr/bin/env python2
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
import time
from GrabParams import grabParams


mc = MyCobot(grabParams.usb_dev, grabParams.baudrate)
#mc.power_on()
mc.set_color(0,0,255)#blue, arm is busy   

coords_bottom_ready = [61.5, 80.8, 220.0, 86.53, 40.7, -178.72]
mc.send_coords(coords_bottom_ready,39,0)
time.sleep(0.8)

mc.set_color(0,255,0)#green, arm is free