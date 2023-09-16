#!/usr/bin/env python

from actionlib.action_client import GoalManager
import rospy
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import re
import os
import tf


def goto_desk():
    os.system("python follow_aruco_2.py")

def pick_color():
    os.system("python color_grab_2.py")

def goal_3():
    os.system("python goto_table.py")

   
def moveback():
    rospy.init_node('robuster_key')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rate = rospy.Rate(20) # 20hz
    print("backward 0.25m...")
    count = 50
    move_cmd = Twist()
    while count > 0:
        move_cmd.linear.x = -0.1
        pub.publish(move_cmd)
        rate.sleep()
        count -= 1
        



def gs_demo():
    goal_3()
    goto_desk()
    pick_color()
    moveback()


    return "Mission Finished."

if __name__ == '__main__':    
    gs_demo()
