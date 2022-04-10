#! /usr/bin/env python
# import rospy
# import time
import math
from tf.transformations import quaternion_from_euler
roll = 0
pitch = 0
yaw = 0.0
while (yaw <= math.pi * 1):
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quat)
    # print(str(yaw) + ': ' + str(quat))
    yaw += math.radians(90.0)
    print ("The quaternion representation is %s %s %s %s." % (quaternion[0], quaternion[1], quaternion[2], quaternion[3]))