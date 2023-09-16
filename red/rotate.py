
import time, math, rospy, tf
from tf.transformations import *
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
done = False
delta_angle = 1000
target_yaw = 2000
current_yaw = 3000
pub = None
listener = None

def rotate_():
    global current_yaw
    global done
    global pub
    delta_angle = target_yaw - current_yaw
    print ('target_yaw: ', target_yaw, 'current_yaw: ', current_yaw, 'delta_angle: ', delta_angle)
    move_cmd = Twist()
    if abs(delta_angle) < 1:
        done = True
    elif delta_angle > 0 and delta_angle < 100:
        move_cmd.angular.z = 0.05 + 0.05 * math.sqrt(0.1 * abs(delta_angle))
    else:
        move_cmd.angular.z = -0.05 - 0.05 * math.sqrt(0.1 * abs(delta_angle))
    pub.publish(move_cmd)


def init_node():
    global listener
    global pub
    rospy.init_node('gsdemo', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    listener = tf.TransformListener()


def getCurrentYaw():
    global current_yaw
    listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(2), None)
    trans, rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    r, p, y = tf.transformations.euler_from_quaternion(rot)
    current_yaw = y * 180 / 3.1415926
    if current_yaw > 0:
        current_yaw = -360 - current_yaw ###
    print(current_yaw)
    return


def read_direct(filename):
    file_to_read = open(filename)
    yaw_target = 0
    for line in file_to_read.readlines():
        line = line.strip()
        yaw_target = float(line)

    return yaw_target

def rotate_to():
    # target_yaw = read_direct(filename)
    # done = False
    init_node()
    while not done:
        getCurrentYaw()
        rotate_()
        time.sleep(0.05)
