#!/usr/bin/env python

from actionlib.action_client import GoalManager
import rospy
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import re
import os
import tf


def goto_shelf_common():
    os.system("python goto_shelf_common.pyc")

def pick_and_place():
    os.system("python pick_and_place.py")

def pick_and_place_myself():
    os.system("python pick_and_place_myself.py")

def goto_shelf_myself():
    os.system("python goto_shelf_myself.pyc")

def goto_start_area():
    os.system("python goto_start_area.pyc")

def color():
    os.system("python color.py")

def gs_demo():
    #goto_shelf_common()
    pick_and_place()
    color()
    #goto_shelf_myself()
    pick_and_place_myself()

    goto_start_area()


    return "Mission Finished."

if __name__ == '__main__':    
    gs_demo()