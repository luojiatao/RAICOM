#!/usr/bin/env python2
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
import time
from GrabParams import grabParams
import basic


mc = MyCobot(grabParams.usb_dev, grabParams.baudrate)

mc.set_color(0,0,255)#blue, arm is busy   

coords_bottom_ready = [61.5, 80.8, 220.0, 86.53, 40.7, -178.72] #61.5, 80.8, 290.0, 86.53, 40.7, -178.72
coords_bottom_grap = [56.5, 153.0, 246.0, 83.53, 40.7, -180.0] #56.5, 170.0, 310.0, 86.53, 40.7, -178.72
coords_bottom_grap_ok = [61.5, 60.8, 260.8, 86.53, 45, -178.72]
mc.send_coords(coords_bottom_grap,39,0)
time.sleep(1.8)
basic.grap(True)
mc.send_coords(coords_bottom_grap_ok,39,0)
coords_place_right = [-150.0, 18.2, 182.6, -177.46, 7.03, 20.66]
mc.send_coords(coords_place_right,39,0)
time.sleep(3.2)  ##4
# open
basic.grap(False)
angles = [0, 0, 0, 0, 0, 0]
mc.send_angles(angles,39)
time.sleep(1.5)
mc.send_coords(coords_bottom_ready,39,0)
time.sleep(1)

mc.set_color(0,255,0)#green, arm is free