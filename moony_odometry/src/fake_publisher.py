#! /usr/bin/env python
import rospy

from localizer_dwm1001.msg import Tag

def talker():
    pub = rospy.Publisher("dwm1001/tag", Tag, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        data = Tag()
        data.x = 1
        data.y = .5
        data.z = .25

        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#     except rospy.ROSInterruptException:
#         print("program interrupted before completion")  # , file=sys.stderr)