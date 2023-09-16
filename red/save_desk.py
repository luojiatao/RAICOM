#!/usr/bin/env python
import os, rospy, tf
from tf.transformations import *
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
rospy.init_node('get_pose', anonymous=True)
listener = tf.TransformListener()
listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(2), None)
trans, rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
r, p, y = tf.transformations.euler_from_quaternion(rot)
os.system('rm goal_4.txt')
with open('goal_4.txt', 'w') as (f):
    f.writelines([str(trans) + '\n', str(rot)])
print ('trans: ', trans, 'rot: ', rot)
