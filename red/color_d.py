#encoding: UTF-8
#!/usr/bin/env python2
import cv2
import os
import numpy as np
import time
#import rospy
#from pymycobot.mycobot import MyCobot
#from opencv_yolo1 import yolo
#from VideoCapture1 import FastVideoCapture
import math
from GrabParams1 import grabParams
#from geometry_msgs.msg import Twist
#import basic
#import argparse

# parser = argparse.ArgumentParser(description='manual to this script')
# parser.add_argument("--debug", type=bool, default="True")
# args = parser.parse_args()


y_bias = grabParams.y_bias 
x_bias = grabParams.x_bias 
height_bias = grabParams.height_bias

IMG_SIZE = grabParams.IMG_SIZE

cap_num = grabParams.cap_num

# show image and waitkey
debug = True 

coords = grabParams.coords_ready
done = grabParams.done




    # choose place to set cube
color = 0
# parameters to calculate camera clipping parameters
x1 = x2 = y1 = y2 = 0        
    # set color HSV
HSV = {
    "yellow": [np.array([20, 43, 46]), np.array([26, 255, 255])],
    "red": [np.array([0, 43, 46]), np.array([10, 255, 255])],
    "green": [np.array([50, 43, 46]), np.array([65, 255, 255])],
    "blue": [np.array([100, 43, 46]), np.array([124, 255, 255])],
    "purple": [np.array([125, 43, 46]), np.array([155, 255, 255])],
}
# use to calculate coord between cube and mycobot
sum_x1 = sum_x2 = sum_y2 = sum_y1 = 0
# The coordinates of the cube relative to the mycobot
c_x, c_y = IMG_SIZE/2, IMG_SIZE/2
# The ratio of pixels to actual values
ratio = grabParams.ratio

# def initialize_rospy():                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
#     rospy.init_node('Detect_marker', anonymous=True)
#     .pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
#     .rate = rospy.Rate(20) # 20hz

# Grasping motion

#机械臂放置积木到右侧


# init mycobot
  


# def get_position(x, y):
#     wx = wy = 0
#     if grabParams.grab_direct == "front":
#         wx = (c_y - y) * ratio
#         wy = (c_x - x) * ratio
#     elif grabParams.grab_direct == "right":
#         wx = (c_x - x) * ratio
#         wy = (y - c_y) * ratio
#     return wx, wy
        
# def transform_frame(frame):
#     frame, ratio, (dw, dh) = yolo.letterbox(frame, (IMG_SIZE, IMG_SIZE))

#     return frame 

# detect cube color
def color_detect(img):
    # set the arrangement of color'HSV
    #img = transform_frame(img)
    x = y = 0
    for mycolor, item in HSV.items():
        redLower = np.array(item[0])
        redUpper = np.array(item[1])
        # transfrom the img to model of gray
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # wipe off all color expect color in range
        mask = cv2.inRange(hsv, item[0], item[1])
        # a etching operation on a picture to remove edge roughness
        erosion = cv2.erode(mask, np.ones((1, 1), np.uint8), iterations=2)
        # the image for expansion operation, its role is to deepen the color depth in the picture
        dilation = cv2.dilate(erosion, np.ones(
            (1, 1), np.uint8), iterations=2)
        # adds pixels to the image
        target = cv2.bitwise_and(img, img, mask=dilation)
        # the filtered image is transformed into a binary image and placed in binary
        ret, binary = cv2.threshold(dilation, 127, 255, cv2.THRESH_BINARY)
        # get the contour coordinates of the image, where contours is the coordinate value, here only the contour is detected
        contours, hierarchy = cv2.findContours(
            dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # do something about misidentification
            boxes = [
                box
                for box in [cv2.boundingRect(c) for c in contours]
                if 110 < min(box[2], box[3]) and  max(box[2], box[3]) < 170
                # if min(img.shape[0], img.shape[1]) / 10
                # < min(box[2], box[3])
                # < min(img.shape[0], img.shape[1]) / 1
            ]
            print(boxes)
            if boxes:
                for box in boxes:
                    # print(box)
                    x, y, w, h = box
                    # if abs(w-h)>15:
                    #     return None
                # find the largest object that fits the requirements
                c = max(contours, key=cv2.contourArea)
                # get the lower left and upper right points of the positioning object
                x, y, w, h = cv2.boundingRect(c)
                print(x, y, w, h)
                # locate the target by drawing rectangle
                cv2.rectangle(img, (x, y), (x+w, y+h), (153, 153, 0), 2)
                # calculate the rectangle center
                x, y = (x*2+w)/2, (y*2+h)/2
                # calculate the real coordinates of mycobot relative to the target
                if mycolor == "yellow":
                    color = 1
                elif mycolor == "red":
                    color = 0
                else:
                    color = 1

    if abs(x) + abs(y) > 0:
        return x, y
    else:
        return None
  
            

   

