#encoding: UTF-8
#!/usr/bin/env python2
import cv2
import os
import numpy as np
import time
#import rospy
from pymycobot.mycobot import MyCobot
from opencv_yolo1 import yolo
#from VideoCapture1 import FastVideoCapture
import math
from GrabParams1 import grabParams
#from geometry_msgs.msg import Twist
import basic
import argparse

parser = argparse.ArgumentParser(description='manual to this script')
parser.add_argument("--debug", type=bool, default="True")
args = parser.parse_args()


y_bias = 45 
x_bias = 0 
height_bias = 150

IMG_SIZE = grabParams.IMG_SIZE

cap_num = grabParams.cap_num

# show image and waitkey
debug = True 

coords = grabParams.coords_ready
# done = False

class Detect_marker(object):
    def __init__(self):
        super(Detect_marker, self).__init__()

        #self.initialize_rospy()
        self.mc = MyCobot(grabParams.usb_dev, grabParams.baudrate)
        self.mc.power_on()

        self.yolo = yolo()
        #self.done = False
        #self.coords = grabParams.coords_ready

        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0        
         # set color HSV
        self.HSV = {
            "yellow": [np.array([20, 43, 46]), np.array([26, 255, 255])],
            "red": [np.array([0, 43, 46]), np.array([10, 255, 255])],
            "green": [np.array([50, 43, 46]), np.array([65, 255, 255])],
            "blue": [np.array([100, 43, 46]), np.array([124, 255, 255])],
            "purple": [np.array([125, 43, 46]), np.array([155, 255, 255])],
        }
        # use to calculate coord between cube and mycobot
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the cube relative to the mycobot
        self.c_x, self.c_y = IMG_SIZE/2, IMG_SIZE/2
        # The ratio of pixels to actual values
        self.ratio = grabParams.ratio

    # def initialize_rospy(self):                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
    #     rospy.init_node('Detect_marker', anonymous=True)
    #     self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    #     self.rate = rospy.Rate(20) # 20hz
    
    # Grasping motion
    def move(self, x, y):
        global height_bias, done
        coords_target = [coords[0]+int(x), coords[1]+int(y), height_bias, coords[3], coords[4], coords[5]]
        basic.move_to_target_coords(coords_target, grabParams.GRAB_MOVE_SPEED)
       

        basic.grap(True)

        angles = [0, 0, 0, 0, 0, 0]
        self.mc.send_angles(angles,30)
        time.sleep(1.5)
        self.place2right()
        #debug = False
        # done = True
        # print("Done")
        # self.mc.set_color(0,255,0)#green, arm is free

    #机械臂放置积木到右侧
    def place2right(self):
        coords_place_right = [-160.0, 18.2, 192.6, -177.46, 7.03, 20.66]
        self.mc.send_coords(coords_place_right,35,0)
        time.sleep(2.5)  ##4

        # open
        basic.grap(False)

        angles = [0, 0, 0, 0, 0, 0]
        self.mc.send_angles(angles,35)
        time.sleep(2)

        angles2 = [-83.23, -140.53, 140.97, 58.71, -127.61, 5.71]
        self.mc.send_angles(angles2,35)
        time.sleep(2.5)
        # done = True
        # print("Done")
        
        self.mc.set_color(0,255,0)#green, arm is free

    # init mycobot
    def init_mycobot(self):
        self.mc.set_color(0,0,255)#blue, arm is busy 
        angles = [0, 0, 0, 0, 0, 0]
        self.mc.send_angles(angles,30)
        basic.grap(False)      
        time.sleep(1)         
        basic.move_to_target_coords(coords,grabParams.GRAB_MOVE_SPEED)     
   

    def get_position(self, x, y):
        wx = wy = 0
        if grabParams.grab_direct == "front":
            wx = (self.c_y - y) * self.ratio
            wy = (self.c_x - x) * self.ratio
        elif grabParams.grab_direct == "right":
            wx = (self.c_x - x) * self.ratio
            wy = (y - self.c_y) * self.ratio
        return wx, wy
            


    def transform_frame(self, frame):
        frame, ratio, (dw, dh) = self.yolo.letterbox(frame, (IMG_SIZE, IMG_SIZE))

        return frame
   
    # detect cube color
    def color_detect(self, img):
        # set the arrangement of color'HSV
        x = y = 0
        #for mycolor, item in self.HSV.items():
        redLower = np.array([0, 43, 46])
        redUpper = np.array([10, 255, 255])
        l_red2 = np.array([[160,43,46]])
        h_red2 = np.array([180,255,255])

        # transfrom the img to model of gray
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # wipe off all color expect color in range
        mask = cv2.inRange(hsv, redLower, redUpper)
        mask2 = cv2.inRange(hsv, l_red2, h_red2)
        #cv2.imshow("figure2", mask)
        # a etching operation on a picture to remove edge roughness
        erosion = cv2.erode(mask, np.ones((1, 1), np.uint8), iterations=2)
        
        # the image for expansion operation, its role is to deepen the color depth in the picture
        dilation = cv2.dilate(erosion, np.ones(
            (1, 1), np.uint8), iterations=2)
        erosion2 = cv2.erode(mask2, np.ones((1, 1), np.uint8), iterations=2)        
        # the image for expansion operation, its role is to deepen the color depth in the picture
        dilation2 = cv2.dilate(erosion2, np.ones(
            (1, 1), np.uint8), iterations=2)
        binary4 = cv2.addWeighted(dilation,1,dilation2,1,0)#两部分红色融合
        #cv2.imshow("figure2", binary4)
        # adds pixels to the image
        target = cv2.bitwise_and(img, img, mask=dilation)
        # the filtered image is transformed into a binary image and placed in binary
        ret, binary = cv2.threshold(target, 127, 255, cv2.THRESH_BINARY)
        #cv2.imshow("figure1", binary)
        # get the contour coordinates of the image, where contours is the coordinate value, here only the contour is detected
        contours, hierarchy = cv2.findContours(
            binary4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #print("-------", contours)
        if len(contours) > 0:
            # # do something about misidentification
            # boxes = [
            #     box
            #     for box in [cv2.boundingRect(c) for c in contours]
            #     if 110 < min(box[2], box[3]) and  max(box[2], box[3]) < 170
            #     # if min(img.shape[0], img.shape[1]) / 10
            #     # < min(box[2], box[3])
            #     # < min(img.shape[0], img.shape[1]) / 1
            # ]
            # print(boxes)
            # if boxes:
            # for box in boxes:
            #     # print(box)
            #     x, y, w, h = box
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
                # if mycolor == "yellow":
                #     self.color = 1
                # elif mycolor == "red":
                #     self.color = 0
                # else:
                #     self.color = 1

        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None

    # def run(self):
    #     self.mc.set_color(0,0,255)#blue, arm is busy
    #     self.init_mycobot()
        

    def show_image(self, img):
        print(args.debug)
        if debug and args.debug:
            cv2.imshow("figure", img)
            cv2.waitKey(50)



            

   

