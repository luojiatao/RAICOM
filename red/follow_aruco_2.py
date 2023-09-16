#encoding: UTF-8
#!/usr/bin/env python2
import cv2 as cv
import os
import numpy as np
import time
from pymycobot.mycobot import MyCobot
from VideoCapture import FastVideoCapture
import math
import rospy
from geometry_msgs.msg import Twist

done = False
cap_num = 0
usb_dev = "/dev/arm"


# show image and waitkey
debug = True 
front_aruco_id = 1
back_aruco_id  = 2

class Follow_aruco(object):
    def __init__(self):
        super(Follow_aruco, self).__init__()

        rospy.init_node('follow_aruco', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(20) # 20hz
        

        # Creating a Camera Object
        self.cap = FastVideoCapture(cap_num)
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv.aruco.DetectorParameters_create()
        self.calibrationParams = cv.FileStorage("calibrationFileName.xml", cv.FILE_STORAGE_READ)
        # Get distance coefficient.
        self.dist_coeffs = self.calibrationParams.getNode("distCoeffs").mat()

        height = self.cap.getHeight()
        focal_length = width = self.cap.getWidth()
        center = [width / 2, height / 2]
        # Calculate the camera matrix.
        self.camera_matrix = np.array(
            [
                [focal_length, 0, center[0]],
                [0, focal_length, center[1]],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )
        self.c_x, self.c_y = width/2, height/2
        self.ratio = 0.25
        self.miss_count = 0
    
   

    # calculate the coords between cube and robot
    def get_position_size(self, corner):
        x = corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]
        y = corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]
        x = x/4.0
        y = y/4.0
        x_size_p = abs(x - corner[0][0])*2
        y_size_p = abs(y - corner[0][1])*2
        return (-(x - self.c_x)), (-(y - self.c_y)), x_size_p, y_size_p


    def show_image(self, img):
        if debug:
            cv.imshow("figure", img)
            cv.waitKey(50) 

    def send_cmd_vel(self, x , y):
        move_cmd = Twist()      

        if y < -23:
            global done
            done = True
            self.pub.publish(Twist())
        else:
            move_cmd.linear.x = 0.15
            if abs(x) > 1:
                move_cmd.angular.z = x/self.cap.getWidth()
            self.pub.publish(move_cmd)
        self.rate.sleep()
        print(move_cmd.linear.x, move_cmd.angular.z)


    def detect_aruco(self, img):
        # transfrom the img to model of gray
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Detect ArUco marker.
        corners, ids, rejectImaPoint = cv.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        self.is_find_front_aruco = False
        self.is_find_back_aruco  = False

        if len(corners) > 0:                
            if ids is not None:
                # get informations of aruco
                ret = cv.aruco.estimatePoseSingleMarkers(
                    corners, 0.021, self.camera_matrix, self.dist_coeffs
                )
                # rvec:rotation offset,tvec:translation deviator
                (rvec, tvec) = (ret[0], ret[1])
                (rvec - tvec).any()
                
                # print corners
                # print rvec.shape[0]

                for i in range(rvec.shape[0]):

                    # draw the aruco on img
                    cv.aruco.drawDetectedMarkers(img, corners)
                    cv.aruco.drawAxis(
                        img,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec[i, :, :],
                        tvec[i, :, :],
                        0.03,
                    )
                    if ids[i][0] == front_aruco_id:
                        self.is_find_front_aruco = True
                        self.front_aruco_info = self.get_position_size(corners[i][0])
                        print("front", self.front_aruco_info)
                    elif ids[i][0] == back_aruco_id:
                        self.is_find_back_aruco  = True
                        self.back_aruco_info = self.get_position_size(corners[i][0])
                        print("back", self.back_aruco_info)
                    
    def follow_single_aruco(self, aruco_info, stop_p):             
        x, y, x_size_p, y_size_p = aruco_info
        size_p =  (x_size_p + y_size_p)/2

        if size_p > stop_p:            
            self.pub.publish(Twist())
            self.rate.sleep()
            return True
        else:
            move_cmd = Twist()
            move_cmd.linear.x = 0.15
            if abs(x) > 1:
                move_cmd.angular.z = x/self.cap.getWidth()
            self.pub.publish(move_cmd)
            print(move_cmd.linear.x, move_cmd.angular.z)
        self.rate.sleep()
        return False

    def follow_back_aruco(self, stop_p):
        x, y, x_size_p, y_size_p = self.back_aruco_info
        size_p =  (x_size_p + y_size_p)/2
        if size_p < 30:
            return False
        return self.follow_single_aruco(self.back_aruco_info,stop_p)
        
    def calc_distance_to_center(self):
        x1, y1, x_size_p1, y_size_p1 = self.front_aruco_info
        x2, y2, x_size_p2, y_size_p2 = self.back_aruco_info
        dx = x2 - x1
        mean_size_p = (x_size_p1 + y_size_p1 + x_size_p2 + y_size_p2)/4.0
        distance_to_center = dx/mean_size_p
        print("distance_to_center", distance_to_center)
        return distance_to_center

    def rotate_to_center(self, aruco_info):
        move_cmd = Twist()      
        is_finised = True
        x, y, x_size_p, y_size_p = aruco_info 
        if abs(x) > 10:
            move_cmd.angular.z = x/self.cap.getWidth()
            if move_cmd.angular.z >= -0.15 and move_cmd.angular.z <= 0:
                move_cmd.angular.z = -0.15
            elif move_cmd.angular.z <= 0.15 and move_cmd.angular.z >= 0:
                move_cmd.angular.z = 0.15
            is_finised = False
            print("rotate_to_center", move_cmd)

        self.pub.publish(move_cmd)
        self.rate.sleep()
        return is_finised

    def wait_cmd_vel(self, t, cmd_vel):
        count = t/0.1
        while count > 0:  
            self.pub.publish(cmd_vel)          
            time.sleep(0.1)
            count = count -1

    def move_to_center(self, distance_to_center):

        direct = distance_to_center/abs(distance_to_center)
        t = abs(distance_to_center)/2
        # if abs(distance_to_center) > 1:
        move_cmd = Twist()
        move_cmd.angular.z = -0.5*direct
        # self.pub.publish(move_cmd)
        self.wait_cmd_vel(t, move_cmd)

        move_cmd = Twist()
        move_cmd.linear.x = 0.15
        # self.pub.publish(move_cmd)
        self.wait_cmd_vel(t, move_cmd)

        move_cmd = Twist()
        move_cmd.angular.z = 0.6*direct
        # self.pub.publish(move_cmd)
        self.wait_cmd_vel(t*1.5, move_cmd)
        

        move_cmd = Twist()
        self.pub.publish(move_cmd)
        time.sleep(0.5)

    def search_aruco(self):
        move_cmd = Twist() 
        move_cmd.angular.z = 0.5
        self.wait_cmd_vel(1, move_cmd)
        self.wait_cmd_vel(0.1, Twist())
        self.miss_count = 0

    def run(self):     

        global done
        while not done:
            img = self.cap.read()

            self.detect_aruco(img)
            
            if self.is_find_front_aruco and self.is_find_back_aruco:                
                print("find 2")
                self.miss_count = 0
                distance_to_center = self.calc_distance_to_center()

                if abs(distance_to_center) >= 0.8:
                    if self.rotate_to_center(self.front_aruco_info ):
                        self.move_to_center(distance_to_center)
                else:
                    self.follow_single_aruco(self.front_aruco_info,1000) 
            elif self.is_find_front_aruco:
                print("find 1 front")
                self.miss_count = 0
                self.follow_single_aruco(self.front_aruco_info,1000)
            elif self.is_find_back_aruco:
                print("find 1 back")                
                if self.follow_back_aruco(38):
                    self.miss_count = 0
                    if self.rotate_to_center(self.back_aruco_info):
                        done = True
                else:
                    self.miss_count += 1
                    if self.miss_count > 40:
                        self.search_aruco()
            else:
                print("find none")
                self.miss_count += 1
                if self.miss_count > 40:
                    self.search_aruco()
                            

            self.show_image(img)
        cv.destroyAllWindows()
        

if __name__ == "__main__":    
    detect = Follow_aruco()
    detect.run()

