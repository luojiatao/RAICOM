# uncompyle6 version 3.9.0
# Python bytecode version base 2.7 (62211)
# Decompiled from: Python 3.7.16 (default, Jan 17 2023, 16:06:28) [MSC v.1916 64 bit (AMD64)]
# Embedded file name: goto_shelf_myself.py
# Compiled at: 2022-08-24 21:19:57
from actionlib.action_client import GoalManager
import rospy
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import re, os

def send_goal(goal_number, goal):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    print('goto table!!!')
    wait = client.wait_for_result(rospy.Duration.from_sec(120.0))
    if not wait:
        print('goto table Failed!!!')
    else:
        print('goto table success!!!')


def read_goal(filename):
    goal = MoveBaseGoal()
    file_to_read = open(filename)
    index = 0
    for line in file_to_read.readlines():
        line = line.strip()
        index += 1
        if index == 1:
            pattern = re.compile('(?<=\\[).*?(?=\\])')
            query = pattern.search(line)
            listFromLine = query.group().split(',')
            goal.target_pose.pose.position.x = float(listFromLine[0])
            goal.target_pose.pose.position.y = float(listFromLine[1])
        if index == 2:
            pattern = re.compile('(?<=\\[).*?(?=\\])')
            query = pattern.search(line)
            listFromLine = query.group().split(',')
            goal.target_pose.pose.orientation.z = float(listFromLine[2])
            goal.target_pose.pose.orientation.w = float(listFromLine[3])

    print(goal.target_pose.pose)
    return goal


def moblie_fetch_demo():
    goal2 = read_goal('goal_3.txt')
    goal_number = 2
    send_goal(goal_number, goal2)
    return 'Finished.'


if __name__ == '__main__':
    rospy.init_node('send_goals_python', anonymous=True)
    result = moblie_fetch_demo()
    rospy.loginfo(result)
# okay decompiling .\goto_shelf_myself.pyc
