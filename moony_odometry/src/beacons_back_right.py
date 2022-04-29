#! /usr/bin/env python
import rospy

from localizer_dwm1001.msg import Tag
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Float64

pub = rospy.Publisher('moony/beacon_back_right', PoseWithCovarianceStamped, queue_size=10)

def func(tag):
    data = PoseWithCovarianceStamped()

    data.header.stamp =  rospy.Time.now()
    data.header.frame_id = "map" # TODO: make this; should it be "map" ?

    data.pose.pose.position.x = tag.x 
    data.pose.pose.position.y = tag.y 
    data.pose.pose.position.z = tag.z 

    data.pose.pose.orientation.x = 0
    data.pose.pose.orientation.y = 0
    data.pose.pose.orientation.z = 0
    data.pose.pose.orientation.w = 1


    data.pose.covariance = [1e-9, 0,    0,    0,    0,    0,
                                0,    1e-9, 0,    0,    0,    0, 
                                0,    0,    1e-9, 0,    0,    0, 
                                0,    0,    0,    1e-9, 0,    0, 
                                0,    0,    0,    0,    1e-9, 0, 
                                0,    0,    0,    0,    0,    1e-9]
    return data

def callback(tagData):
    global outData
    global pub  # rospy.loginfo(rospy.get_caller_id() + "I heard %s", tagData.data)
    
    outData = func(tagData)
    pub.publish(outData)
    
def listener():
    global outData
  
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("dwm1001/tagSecond", Tag, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
