#!/usr/bin/env python
import rospy
# from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

robotStart = PoseStamped()
robotGoal = PoseStamped()

frame = "skid_drive"  # ensure this matches the move base package's global frame
# robotStart.header.seq = 0
# robotStart.header.stamp = rospy.Time(0)
robotStart.header.frame_id = frame
robotGoal.pose.position.x = 0.0
robotGoal.pose.position.y = 0.0
robotGoal.pose.position.z = 0.0
robotStart.pose.orientation.x = 0.0
robotStart.pose.orientation.y = 0.0
robotStart.pose.orientation.w = 1.0

robotGoal.header.seq = 0
# robotGoal.header.stamp = rospy.Time(0)
robotGoal.header.frame_id = frame
robotGoal.pose.position.x = 2 #1.58
robotGoal.pose.position.y = 0.0 #1.58
robotGoal.pose.position.z = 0.0
robotGoal.pose.orientation.x = 0.0
robotGoal.pose.orientation.y = 0.0
robotGoal.pose.orientation.w = 1.0

srvPlan = GetPlanRequest()
srvPlan.start = robotStart
srvPlan.goal = robotGoal
srvPlan.tolerance = 1.5

print("loc: %.2f, %.2f -> goal: %.2f, %.2f" % (srvPlan.start.pose.position.x,
      srvPlan.start.pose.position.y, srvPlan.goal.pose.position.x, srvPlan.goal.pose.position.y))


def client():
    rospy.wait_for_service('move_base/make_plan')
    try:
        print("inside client")
        serviceHandle = rospy.ServiceProxy('move_base/make_plan', GetPlan)  # Request)
        # rospy.ServiceProxy

        pathPlanResponse = serviceHandle.call(srvPlan)
        print("Type of pathPlanResponse: \n%s" % type(pathPlanResponse))
        myPath = pathPlanResponse.plan
        print("Type of myPath: \n%s" % type(myPath))
        myPath.header.frame_id = frame
        print(myPath.header)
        # print(myPath.poses)
        print(len(myPath.poses))
        # path2 = GetPlanResponse(serviceHandle.call(srvPlan)).plan) # this is the WRONG way to construct a response; just allow the service handle to return it
        
        pub = rospy.Publisher('chatter', Path, queue_size=100)
        i = 0.0
        rate = rospy.Rate(10) #10 hz
        while (i < 100):
            pub.publish(myPath)
            rate.sleep()
            i += 1
            

        # thePath = Path(pathPlanResponse.plan) # error, needs args: header, poses
        # print(len(pathPlanResponse.plan))
        # for i in pathPlanResponse.plan:
        # print(i)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False


if __name__ == "__main__":
    # if len(sys.argv) == 3:
    rospy.init_node('navigate_client_py')
    print("Requesting ")
    client()

# test
