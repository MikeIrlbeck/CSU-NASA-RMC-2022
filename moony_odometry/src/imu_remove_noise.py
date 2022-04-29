#! /usr/bin/env python
import rospy

from sensor_msgs.msg import Imu

pub = rospy.Publisher('moony/imu_smooth', Imu, queue_size=10)

def func(imuNoisy):
    data = imuNoisy

    accelX = imuNoisy.linear_acceleration.x
    accelY = imuNoisy.linear_acceleration.y

    noiseThreshold = 0.5

    if (abs(accelX) < noiseThreshold):
        accelX = 0
    if (abs(accelY) < noiseThreshold):
        accelY = 0
    

    data.linear_acceleration.x = accelX
    data.linear_acceleration.y = accelY

    return data

def callback(imuNoisyData):
    global outData
    global pub  # rospy.loginfo(rospy.get_caller_id() + "I heard %s", imuNoisyData.data)
    
    outData = func(imuNoisyData)
    pub.publish(outData)
    
def listener():
    global outData
  
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("moony/imu", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
