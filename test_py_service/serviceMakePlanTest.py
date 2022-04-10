#!/usr/bin/env python
import rospy
# from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

# rospy.init_node('navigate_client_py')

robotStart = PoseStamped()
robotGoal = PoseStamped()

robotStart.header.seq = 0
robotStart.header.stamp = rospy.Time(0)
robotStart.header.frame_id = "map"
robotGoal.pose.position.x = 0.0
robotGoal.pose.position.y = 0.0
robotGoal.pose.position.z = 0.0
robotStart.pose.orientation.x = 0.0
robotStart.pose.orientation.y = 0.0
robotStart.pose.orientation.w = 1.0

robotGoal.header.seq = 0
robotGoal.header.stamp = rospy.Time(0)
robotGoal.header.frame_id = "map"
robotGoal.pose.position.x = 1.0
robotGoal.pose.position.y = 0.0
robotGoal.pose.position.z = 0.0
robotGoal.pose.orientation.x = 0.0
robotGoal.pose.orientation.y = 0.0
robotGoal.pose.orientation.w = 1.0

srvPlan = GetPlan()
srvPlan.Request.start = robotStart
srvPlan.Request.goal = robotGoal
srvPlan.Request.tolerance = 1.5


def client():
    rospy.wait_for_service('/move_base/make_plan')
    try:
        serviceHandle = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        pathPlan = serviceHandle.call(srvPlan)

        # for i in pathPlan.
        print(pathPlan.poses)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False


if __name__ == "__main__":
    # if len(sys.argv) == 3:

    print("Requesting ")  # %s+%s"%(x, y))
    client()
    # print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))

# test
