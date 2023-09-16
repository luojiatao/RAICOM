#encoding: UTF-8
#!/usr/bin/env python2
import cv2 as cv
import os
import numpy as np
import time
from pymycobot.mycobot import MyCobot
from follow_obj import followObj
from opencv_yolo import yolo
from VideoCapture import FastVideoCapture
from GrabParams2 import grabParams
import math
import rospy
from geometry_msgs.msg import Twist
import basic
import argparse
import rotate
import color_de
import a_follow

#0：'apple', 1：'clock', 2：'banana'
top_pick_class = 0

choose_all_class = False

obj_size_filter = 80

follow_forward_target_obj_size = 108


coords_forward_ready = [80.6, -61.0, 300.9, 86.7, 42.72, 90.5]

#仓库上层货架抓取，机械臂的控制位置，可以通过注释其他代码进行调试
coords_top_ready_myself = [61.5, 80.8, 280.0, 86.53, 40.7, -178.72]
coords_top_grap_myself = [56.5, 171.0, 320.0, 85.0, 40.7, -180.0] #[56.5, 170.0, 315.0, 86.53, 40.7, -178.72]
coords_top_grap_ok_myself = [61.5, 60.8, 330.8, 86.53, 45, -178.72]

#仓库下层货架抓取，机械臂的控制位置，可以通过注释其他代码进行调试
coords_bottom_ready_myself = [63.5, 80.8, 212.0, 89.53, 40.7, -178.72] #61.5, 80.8, 290.0, 86.53, 40.7, -178.72
coords_bottom_grap_myself = [56.5, 174.0, 220.0, 84.53, 40.7, -180.0] #56.5, 170.0, 310.0, 86.53, 40.7, -178.72
coords_bottom_grap_ok_myself = [61.5, 60.8, 260.8, 86.53, 45, -178.72]

#xsize,ysize >=128
#上层货架抓取，机械臂的控制位置，可以通过注释其他代码进行调试
coords_top_ready = [61.5, 80.8, 295.0, 86.53, 40.7, -178.72]
coords_top_grap = [56.5, 152.0, 347.0, 83.53, 40.7, -180.0] #[56.5, 170.0, 315.0, 86.53, 40.7, -178.72]
coords_top_grap_ok = [61.5, 60.8, 350.8, 86.53, 45, -178.72]

#下层货架抓取，机械臂的控制位置，可以通过注释其他代码进行调试
coords_bottom_ready = [61.5, 80.8, 220.0, 86.53, 40.7, -178.72] #61.5, 80.8, 290.0, 86.53, 40.7, -178.72
coords_bottom_grap = [56.5, 153.0, 246.0, 83.53, 40.7, -180.0] #56.5, 170.0, 310.0, 86.53, 40.7, -178.72
coords_bottom_grap_ok = [61.5, 60.8, 260.8, 86.53, 45, -178.72]
arm_speed = 35

find_obj_count_top_max = 3
find_obj_count_bottom_max = 5
find_obj_distance_max_top = 0.45
find_obj_distance_max_bottom = 0.50

y_bias = 45 
x_bias = 0
height_bias = grabParams.height_bias1
#coords = grabParams.coords_ready1

class gsdemo(object):
    #初始化函数
    def __init__(self):
        super(gsdemo, self).__init__()
        
        self.initialize_robotarm()

        self.initialize_rospy()
        
        self.initialize_vision()  

        self.initialize_followObj()      
        
        self.initialize_find_obj_info()

        # #加载模型
        # self.net = cv.dnn.readNetFromONNX("comp.onnx")

        # self.color = 0
        # # parameters to calculate camera clipping parameters
        # self.x1 = self.x2 = self.y1 = self.y2 = 0        
        #  # set color HSV
        # self.HSV = {
        #     "yellow": [np.array([20, 43, 46]), np.array([26, 255, 255])],
        #     "red": [np.array([0, 43, 46]), np.array([10, 255, 255])],
        #     "green": [np.array([50, 43, 46]), np.array([65, 255, 255])],
        #     "blue": [np.array([100, 43, 46]), np.array([124, 255, 255])],
        #     "purple": [np.array([125, 43, 46]), np.array([155, 255, 255])],
        # }
        # # use to calculate coord between cube and mycobot
        # self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # self.ratio1 = grabParams.ratio1

    def initialize_robotarm(self):
        self.mc = MyCobot(grabParams.usb_dev, grabParams.baudrate)
        self.mc.power_on()

    def initialize_rospy(self):                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        rospy.init_node('gsdemo', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(20) # 20hz

    def initialize_vision(self):
        self.cap = FastVideoCapture(grabParams.cap_num)       
        self.c_x, self.c_y = grabParams.IMG_SIZE/2, grabParams.IMG_SIZE/2

        self.colordetect = color_de.Detect_marker()
        self.aruco_follow = a_follow.Follow_aruco()      

        self.yolo = yolo()

        height = grabParams.IMG_SIZE                                                                                                                                                                            
        focal_length = width = grabParams.IMG_SIZE
        self.center = [width / 2, height / 2]
                                                                                            
    def initialize_followObj(self):
        self.followObj = followObj(self.pub, self.rate)                          

    def initialize_find_obj_info(self):
        self.find_obj_count_top = 0
        self.find_obj_count_bottom = 0
        self.find_obj_distance_top = 0
        self.find_obj_distance_bottom = 0

    def increase_find_obj_count_top(self):
        self.find_obj_count_top += 1

    def increase_find_obj_count_bottom(self):
        self.find_obj_count_bottom += 1

    def increase_find_obj_distance_top(self):
        self.find_obj_distance_top += 0.0501

    def increase_find_obj_distance_bottom(self):
        self.find_obj_distance_bottom += 0.0501

    def is_find_obj_finish_top(self):
        is_finish = False
        if self.find_obj_distance_top > find_obj_distance_max_top:
            is_finish = True
        return is_finish

    def is_find_obj_finish_bottom(self):
        is_finish = False
        if self.find_obj_distance_bottom > find_obj_distance_max_bottom:
            is_finish = True
        return is_finish
    

    #计算物体的坐标和尺寸
    def get_position_size(self, box):
        left, top, right, bottom = box
        # print("box",box)
        x = left + right
        y = bottom + top
        x = x*0.5
        y = y*0.5
        x_size_p = abs(left - right)
        y_size_p = abs(top - bottom)
        return (-(x - self.c_x)), (-(y - self.c_y)), x_size_p, y_size_p

    #视频显示
    def show_image(self, img):
        cv.imshow("figure", img)
        cv.waitKey(50) 
    
    #物体识别
    def obj_detect(self, img):
        self.is_find = False 
        img_ori = img
        img_ori = self.transform_frame(img)
        img = self.transform_frame_128(img)

        #加载模型
        net = cv.dnn.readNetFromONNX("comp.onnx")
        
        t1 = time.time()
        #输入数据处理
        blob = cv.dnn.blobFromImage(img, 1 / 255.0, (128, 128), [0, 0, 0], swapRB=True, crop=False)
        net.setInput(blob)
        
        #推理
        outputs = net.forward(net.getUnconnectedOutLayersNames())[0]

        
        #获得识别结果
        boxes, classes, scores = self.yolo.yolov5_post_process_simple(outputs)
        t2 = time.time()

        best_result = (0,0,0)
        
        #物体边框处理
        if boxes is not None:
            boxes = boxes*5
            for box, score, cl in zip(boxes, scores, classes):
                self.image_info = self.get_position_size(box)
                wph = self.image_info[2]/self.image_info[3]
                obj_size = (self.image_info[2]+self.image_info[3])*0.5
                # print(cl, wph, self.image_info)

                if (wph > 0.8 and wph <1.2) and (obj_size >= obj_size_filter):
                    if (top_pick_class == cl) or (choose_all_class):
                        temp_result = (box, score, cl)
                        best_score = best_result[1]
                        # print("best_score: ", best_score)
                        if best_score < 0.01:
                            best_result = temp_result
                            self.is_find = True                    
                        best_box =  best_result[0]
                        # print("best_box:", best_box, "box: ", box)
                        temp_box_left, temp_box_top, temp_box_right, temp_box_bottom = box 
                        best_box_left, best_box_top, best_box_right, best_box_bottom = best_box                  
                        if temp_box_left < best_box_left:
                            best_result = temp_result
                            self.is_find = True
        
        #画物体边框
        if self.is_find:                
            box, score, cl = best_result
            self.yolo.draw_single(img_ori, box, score, cl)            
            self.target_image_info = self.get_position_size(box)
            print(self.target_image_info)

        #视频显示
        self.show_image(img_ori)
  

    #图像处理，适配物体识别
    def transform_frame(self, frame):
        frame, ratio, (dw, dh) = self.yolo.letterbox(frame, (grabParams.IMG_SIZE, grabParams.IMG_SIZE))

        return frame
        
    #图像处理，适配物体识别
    def transform_frame_128(self, frame):
        frame, ratio, (dw, dh) = self.yolo.letterbox(frame, (128, 128))

        return frame


    ###颜色识别
       
    def pick_and_place_color(self):
        self.colordetect.init_mycobot()
        done = False
        while not done:
            img = self.cap.read() 
            
            frame = self.transform_frame(img)
            detect_result = self.colordetect.color_detect(frame)
            self.colordetect.show_image(frame)
            if detect_result is None:
                continue
            else:
                x, y = detect_result
                real_x, real_y = self.colordetect.get_position(x, y)
                coords_now = basic.get_coords()
                if len(coords_now) == 6:
                    self.colordetect.coords = coords_now
                self.colordetect.move(real_x + x_bias, real_y + y_bias)
                # time.sleep(9.5)
                done = True
                print("Done")            
    
    ####


    #小车后退
    def moveback(self):
        print("backward 0.05m...")
        count = 40
        move_cmd = Twist()
        while count > 0:
            move_cmd.linear.x = -0.025
            self.pub.publish(move_cmd)
            self.rate.sleep()
            count -= 1

    def moveforward(self):
        print("forward 0.05m...")
        count = 40
        move_cmd = Twist()
        while count > 0:
            move_cmd.linear.x = 0.025
            self.pub.publish(move_cmd)
            self.rate.sleep()
            count -= 1

    def move30cm(self,n,speed):
        print("move 0.3m...")
        count = n
        move_cmd = Twist()
        while count > 0:
            move_cmd.linear.x = speed
            self.pub.publish(move_cmd)
            self.rate.sleep()
            count -= 1
    
    #小车右转
    def rotate_to_right(self):
        print("rotate_to_right...")
        count = 20
        move_cmd = Twist()
        while count > 0:
            move_cmd.angular.z = -0.2
            self.pub.publish(move_cmd)
            self.rate.sleep()
            count -= 1
    
    #小车左转
    def rotate_to_left(self):
        print("rotate_to_left...")
        count = 20
        move_cmd = Twist()
        while count > 0:
            move_cmd.angular.z = 0.2
            self.pub.publish(move_cmd)
            self.rate.sleep()            
            count -= 1

    #夹爪闭合
    def pick(self): 
        basic.grap(True)

    #旋转图像
    def rotate_image(self, image, rotate_angle):
        rotated_image = self.yolo.rotate_image(image,rotate_angle)
        return rotated_image


    #跟随物体
    def follow_obj(self):
        is_follow_obj_done = False

        while not is_follow_obj_done:
            k=cv.waitKey(1)
            if k==ord('q'):
                print('done')
                break 
            img = self.cap.read()            
            img = self.rotate_image(img, -90)
            self.obj_detect(img)           
            if self.is_find:
                # print("send_cmd_vel")
                is_follow_obj_done = self.followObj.follow(self.target_image_info, self.cap.getWidth())
                print(is_follow_obj_done)
            else:
                move_cmd = Twist()
                self.pub.publish(Twist()) 


    #跟随物体
    def follow_obj_forward(self):
        is_follow_obj_done_forward = False

        while not is_follow_obj_done_forward:
            k=cv.waitKey(1)
            if k==ord('q'):
                print('done')
                break 
            img = self.cap.read()            
            img = self.rotate_image(img, -90)
            self.obj_detect(img)           
            if self.is_find:
                is_follow_obj_done_forward = self.followObj.follow_forward(self.target_image_info, self.cap.getWidth(), follow_forward_target_obj_size)
            else:
                move_cmd = Twist()
                self.pub.publish(Twist()) 
    

    #寻找指定的物体
    def find_target_image(self):        
        while not self.is_find:  
            img = self.cap.read()            
            img = self.rotate_image(img, -90)
            self.obj_detect(img) 
            k=cv.waitKey(1)
            if k==ord('q'):
                print('done')
                break  

    #机械臂放置积木到右侧
    def place2right(self):
        coords_place_right = [-150.0, 18.2, 182.6, -177.46, 7.03, 20.66]
        self.mc.send_coords(coords_place_right,arm_speed,0)
        time.sleep(4)  ##4

        # open
        basic.grap(False)

        angles = [0, 0, 0, 0, 0, 0]
        self.mc.send_angles(angles,arm_speed)

    def ready_arm_pose_forward(self):
        global choose_all_class, obj_size_filter 
        choose_all_class = False
        obj_size_filter = 30         
        self.mc.send_coords(coords_forward_ready,arm_speed,0)
        basic.grap(False)
        time.sleep(0.5)

    def ready_arm_pose(self,choose,coords): 
        global choose_all_class, obj_size_filter 
        choose_all_class = choose 
        obj_size_filter = 80      
        self.mc.send_coords(coords,arm_speed,0)
        basic.grap(False)
        time.sleep(0.5)

    # #机械臂到达准备抓取的位置
    # def ready_arm_pose_top(self): 
    #     global choose_all_class, obj_size_filter 
    #     choose_all_class = False 
    #     obj_size_filter = 80      
    #     self.mc.send_coords(coords_top_ready,arm_speed,0)
    #     basic.grap(False)
    #     time.sleep(3)

    # #机械臂到达准备抓取的位置
    # def ready_arm_pose_bottom(self): 
    #     global choose_all_class, obj_size_filter 
    #     choose_all_class = True 
    #     obj_size_filter = 80   
    #     self.mc.send_coords(coords_bottom_ready,arm_speed,0)
    #     basic.grap(False)
    #     time.sleep(3)

    #机械臂进行抓取
    def pick_(self,coords1,coords2):
        
        self.mc.send_coords(coords1,arm_speed,0)
        time.sleep(2)  ##old: 3
        basic.grap(True)#闭合夹爪

        self.mc.send_coords(coords2,arm_speed,0)
        time.sleep(2)

    # #机械臂进行抓取
    # def pick_top(self):
        
    #     self.mc.send_coords(coords_top_grap,arm_speed,0)
    #     time.sleep(2)  ##lsw old: 3
    #     basic.grap(True)#闭合夹爪

    #     self.mc.send_coords(coords_top_grap_ok,arm_speed,0)
    #     time.sleep(2)
        

    # #机械臂进行抓取
    # def pick_bottom(self):
        
    #     self.mc.send_coords(coords_bottom_grap,arm_speed,0)
    #     time.sleep(2)  ##lsw old: 3
    #     basic.grap(True)#闭合夹爪

    #     self.mc.send_coords(coords_bottom_grap_ok,arm_speed,0)
    #     time.sleep(2)

    def follow_obj_and_pick_and_place(self,coords1,coords2):
        # #跟随物体
        self.follow_obj()    
        
        # #机械臂进行抓取
        self.pick_(coords1,coords2)
        
        # #机械臂放置积木到右侧
        self.place2right()

    # def follow_obj_and_pick_and_place_bottom_myself(self):
    #     # #跟随物体
    #     self.follow_obj()    
        
    #     # #机械臂进行抓取
    #     self.pick_(coords_bottom_grap_myself,coords_bottom_grap_ok_myself)
        
    #     # #机械臂放置积木到右侧
    #     self.place2right()
    #     # #中间防碰
    #     time.sleep(2)

    # def follow_obj_and_pick_and_place_top(self):
    #     # #跟随物体
    #     self.follow_obj()    
        
    #     # #机械臂进行抓取
    #     self.pick_(coords_top_grap,coords_top_grap_ok)
        
    #     # #机械臂放置积木到右侧
    #     self.place2right()

    # def follow_obj_and_pick_and_place_bottom(self):
    #     # #跟随物体
    #     self.follow_obj()    
        
    #     # #机械臂进行抓取
    #     self.pick_(coords_bottom_grap,coords_bottom_grap_ok)
        
    #     # #机械臂放置积木到右侧
    #     self.place2right()
    #     # #中间防碰
    #     time.sleep(1.5)
    #     #bottom_ready_middle = [-101.8, -27.1, 223.4, 83.72, 43.39, -40.52]
    #     #self.mc.send_coords(bottom_ready_middle,arm_speed,0)

    def find_next_obj(self):
        img = self.cap.read()            
        img = self.rotate_image(img, -90)
        self.obj_detect(img)           
        if self.is_find:
            self.find_next_obj_count += 1
        else:
            self.find_next_obj_count = 0

    def pick_and_place_objs_top_myself(self): 
        self.ready_arm_pose(False,coords_top_ready_myself)
        self.find_next_obj_count = 0 
        count = 0      
        while not self.is_find_obj_finish_top():
            self.find_next_obj()
            count += 1
            if self.find_next_obj_count >= 5:
                # self.rotate_to_direct2()
                self.increase_find_obj_count_top()
                print("find_obj_count_top: ", self.find_obj_count_top)
                time.sleep(1)
                self.follow_obj_and_pick_and_place(coords_top_grap_myself,coords_top_grap_ok_myself)
                self.ready_arm_pose(False,coords_top_ready_myself)
                time.sleep(2)
                count = 6
            if count >= 6:
                self.moveforward()
                # time.sleep(1)
                count = 0
                self.find_next_obj_count = 0 
                self.increase_find_obj_distance_top()
                print("find_obj_distance_top: ", self.find_obj_distance_top)

    def pick_and_place_objs_bottom_myslf(self):
        self.ready_arm_pose(True,coords_bottom_ready_myself)
        self.find_next_obj_count = 0
        count = 0 
        while not self.is_find_obj_finish_bottom():            
            self.find_next_obj()
            count += 1
            if self.find_next_obj_count >= 5:
                # self.rotate_to_direct2()
                self.increase_find_obj_count_bottom()
                print("find_obj_count_bottom: ", self.find_obj_count_bottom)
                time.sleep(1)
                self.follow_obj_and_pick_and_place(coords_bottom_grap_myself,coords_bottom_grap_ok_myself)
                time.sleep(2)
                self.ready_arm_pose(True,coords_bottom_ready_myself)
                time.sleep(2)
                count = 6
            if count >= 6:
                self.moveback()
                # time.sleep(1)
                count = 0
                self.find_next_obj_count = 0 
                self.increase_find_obj_distance_bottom()
                print("find_obj_distance_bottom: ", self.find_obj_distance_bottom)

    def pick_and_place_objs_top(self): 
        self.ready_arm_pose(False,coords_top_ready)
        self.find_next_obj_count = 0 
        count = 0      
        while not self.is_find_obj_finish_top():
            self.find_next_obj()
            count += 1
            if self.find_next_obj_count >= 5:
                # self.rotate_to_direct2()
                self.increase_find_obj_count_top()
                print("find_obj_count_top: ", self.find_obj_count_top)
                time.sleep(1)
                self.follow_obj_and_pick_and_place(coords_top_grap,coords_top_grap_ok)
                self.ready_arm_pose(False,coords_top_ready)
                time.sleep(2)
                count = 6
            if count >= 6:
                self.moveforward()
                # time.sleep(1)
                count = 0
                self.find_next_obj_count = 0 
                self.increase_find_obj_distance_top()
                print("find_obj_distance_top: ", self.find_obj_distance_top)

    def pick_and_place_objs_bottom(self):
        self.ready_arm_pose(True,coords_bottom_ready)
        self.find_next_obj_count = 0
        count = 0 
        while not self.is_find_obj_finish_bottom():            
            self.find_next_obj()
            count += 1
            if self.find_next_obj_count >= 5:
                # self.rotate_to_direct2()
                self.increase_find_obj_count_bottom()
                print("find_obj_count_bottom: ", self.find_obj_count_bottom)
                time.sleep(1)
                self.follow_obj_and_pick_and_place(coords_bottom_grap,coords_bottom_grap_ok)
                time.sleep(1.5)
                self.ready_arm_pose(True,coords_bottom_ready)
                time.sleep(2)
                count = 6
            if count >= 6:
                self.moveback()
                # time.sleep(1)
                count = 0
                self.find_next_obj_count = 0 
                self.increase_find_obj_distance_bottom()
                print("find_obj_distance_bottom: ", self.find_obj_distance_bottom)

    def arm_back(self):
        angles1 = [0, 0, 0, 0, 0, 0]
        self.mc.send_angles(angles1,35)
        time.sleep(2)

        angles2 = [-83.23, -140.53, 140.97, 58.71, -127.61, 5.71]
        self.mc.send_angles(angles2,35)
        time.sleep(2.5)

        self.mc.set_color(0,255,0)#green, arm is free
    
    def rotate_to_direct(self,filename):
        rotate.target_yaw = rotate.read_direct(filename)
        rotate.done = False
        rotate.rotate_to()

    # def rotate_to_direct2(self):
    #     rotate.target_yaw = rotate.read_direct('direct_2.txt')
    #     rotate.done = False
    #     rotate.rotate_to()
    # def rotate_to_direct1(self):
    #     os.system("python rotate_to_direct1.pyc")

    # def rotate_to_direct2(self):
    #     os.system("python rotate_to_direct2.pyc")

    def goto_shelf_common(self):
        os.system("python goto_shelf_common.pyc")

    def goto_shelf_myself(self):
        os.system("python goto_shelf_myself.pyc")

    def goto_start(self):
        os.system("python goto_start_area.pyc")

    def goto_desk(self):
        self.aruco_follow.run()
        #os.system("python follow_aruco_2.py")

    # def color_grab(self):
    #     os.system("python color_grab_2.py")

    def goto_table(self):
        os.system("python goto_table.py")
    
    def goto_objs(self):

        self.ready_arm_pose_forward()

        self.rotate_to_direct('direct_1.txt')
        
        self.follow_obj_forward()

        self.follow_obj_forward()

        self.follow_obj_forward()

        self.rotate_to_direct('direct_2.txt')


    def pick_common(self):
        self.goto_shelf_common()

        self.mc.set_color(0,0,255)#blue, arm is busy 

        self.goto_objs()
        
        self.pick_and_place_objs_top()

        self.pick_and_place_objs_bottom()  #bottom,4.3

        self.move30cm(50,0.1)

        self.arm_back()

        #cv.destroyAllWindows()

        #cv.destroyWindow("figure")

    def pick_color(self):
        # self.ready_arm_pose_forward()
        # time.sleep(5)
        # self.follow_obj_forward()
        # time.sleep(5)
        # cv.destroyWindow("figure")
        self.goto_table()
        self.goto_desk()
        self.pick_and_place_color()
        self.move30cm(55,-0.1)
    
    def pick_myself(self):
        self.goto_shelf_myself()

        self.mc.set_color(0,0,255)#blue, arm is busy 

        self.goto_objs()

        self.initialize_find_obj_info()
        
        self.pick_and_place_objs_top_myself()

        self.pick_and_place_objs_bottom_myslf()  #bottom,4.3

        self.move30cm(50,0.1)

        self.arm_back()

        cv.destroyAllWindows()
                
            

    #主函数
    def run(self):  
        self.pick_common()
        self.pick_color()
        self.pick_myself()
        self.goto_start()


if __name__ == "__main__":    
    detect = gsdemo()
    detect.run()

