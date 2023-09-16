#!/usr/bin/env python

from actionlib.action_client import GoalManager
import rospy
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import re
import os

def send_goal(goal_number, goal):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    str_log = "Send NO. %s Goal !!!" %str(goal_number)
    rospy.loginfo(str_log)

    wait = client.wait_for_result(rospy.Duration.from_sec(120.0))  
    if not wait:
        str_log="The NO. %s Goal Planning Failed for some reasons" %str(goal_number)
        rospy.loginfo(str_log)
    else:
        str_log="The NO. %s Goal achieved success !!!" %str(goal_number)
        rospy.loginfo(str_log)

def read_goal(filename):
    goal = MoveBaseGoal() 

    file_to_read = open(filename)
    index = 0
    for line in file_to_read.readlines():
        line = line.strip()
        index += 1
        if index == 2:
            pattern = re.compile(r"(?<=\[).*?(?=\])")
            query = pattern.search(line)
            listFromLine = query.group().split(',')
            goal.target_pose.pose.position.x = float(listFromLine[0])
            goal.target_pose.pose.position.y = float(listFromLine[1])
        if index == 3:
            pattern = re.compile(r"(?<=\[).*?(?=\])")
            query = pattern.search(line)
            listFromLine = query.group().split(',')
            goal.target_pose.pose.orientation.z = float(listFromLine[2])
            goal.target_pose.pose.orientation.w = float(listFromLine[3])


    print(goal.target_pose.pose)
    
    return goal

def goto_desk():
    os.system("python /home/robuster/beetle_ai/scripts/follow_aruco_2.py")

def pick_aruco():
    os.system("python color_grab_2.py")

# def place():
#     os.system("python /home/robuster/beetle_ai/scripts/place_2.py")

# def init_arm():
#     os.system("python /home/robuster/beetle_ai/scripts/place_2.py")

def backward():
    print("backward...")
    move_cmd = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    count = 20
    rate = rospy.Rate(count) # 20hz      


    while count > 0:
        move_cmd.linear.x = -0.2
        pub.publish(move_cmd)
        rate.sleep()
        print("backward...")
        count -= 1

def moblie_fetch_demo():
    goal1 = read_goal("goal_4.txt")
    #goal2 = read_goal("goal_2.txt")
    # init_arm()

    # pick cube from goal1
    goal_number = 1
    send_goal(goal_number,goal1)
    goto_desk()
    pick_aruco()
    backward()

    # # place cube to goal2
    # goal_number = 2
    # send_goal(goal_number,goal2)
    # goto_desk()
    # place()  
    # backward() 

    # # pick cube from goal2
    # goal_number = 2
    # send_goal(goal_number,goal2)
    # goto_desk()
    # pick_aruco()
    # backward()

    # # place cube to goal1
    # goal_number = 1
    # send_goal(goal_number,goal1)
    # goto_desk()
    # place()  
    # backward() 
 



    return "Mission Finished."

if __name__ == '__main__':
        rospy.init_node('send_goals_python',anonymous=True)    
        result = moblie_fetch_demo()
        rospy.loginfo(result)
