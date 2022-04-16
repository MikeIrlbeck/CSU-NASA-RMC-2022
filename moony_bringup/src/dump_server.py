#! /usr/bin/env python
import math
import rospy
import actionlib
from std_msgs.msg import String

# Example of importing action messages
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from moony_autonomy_dump.msg import DumpAction, DumpGoal, DumpResult, DumpFeedback

# Example of importing service messges
# for make a plan service
# from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse

# Example of importing general messages
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Odometry, Path

# Another way to import messages
# import geometry_msgs.msg # imports 'all' messsages in this; need to access them via geometry_msgs.msg.Path


class DumpingAction(object):
    def __init__(self, name):
        self.startUp()
        self._action_name = name
        self._actionServer = actionlib.SimpleActionServer(
            self._action_name, DumpAction, execute_cb=self.execute_cb, auto_start=False)
        self._actionServer.start()

    def startUp(self):
        self._result = DumpResult()
        print("Server ACTIVE")

    # Example of sending a goal to another action server
    # def sendNavigation(self, x, y, yaw):
    #     self.locationGoal.target_pose.pose.position.x = x
    #     self.locationGoal.target_pose.pose.orientation.z = quaternion[2]
    #     self.MoveBaseClient.send_goal(self.locationGoal, feedback_cb=self.feedback_movebase)
    #     self.MoveBaseClient.wait_for_result()
    #     return self.MoveBaseClient.get_result()
        # Example of obtaining feedback from the MoveBaseClient action call
    # def feedback_movebase(self, feedback):
    #     print('[Feedback] Going to Goal Pose...')
    #     print(feedback)

    # Example of a 'service' call to a 'service' server
    # def use_this_service(self, var):
    #     # Create a service proxy
    #     makePlanServiceHandle = rospy.ServiceProxy('move_base/make_plan', GetPlan)

    #     # Create a service request and set its fields
    #     makePlanSrvReq = GetPlanRequest()
    #     makePlanSrvReq.start = robotStart

    #     # Send the request and obtain the response
    #     pathPlanResponse = makePlanServiceHandle.call(makePlanSrvReq)
    #     myPath = pathPlanResponse.plan
    #     myPath.header.frame_id = self.frame

    # def subscribe_and_do_something(self, goalList):

    def goal1(self, distance):
        fb = DumpFeedback("doing goal 1: %d" % distance)
        self._actionServer.publish_feedback(fb)

            # Example of a simple publisher
        pub = rospy.Publisher('chatter', String, queue_size=100)
        myString = "hello"
        i = 0.0
        rate = rospy.Rate(10)  # 10 hz
        while (i < 3):
            pub.publish(myString)
            rate.sleep()
            i += 1
        if 1 == 1:
            self._result = DumpResult(True)

    def goal2(self):
             # subscribe to only one message
        self.noise = rospy.wait_for_message("goal2Topic", String, timeout=None)
        print(self.noise)

        self._result = DumpResult(True)

    def execute_cb(self, goal):
        self._result = DumpResult(False)  # only give a successful result if everything went well

        print(goal)
        # if self._actionServer.is_preempt_requested(): # allows the client to cancel the goal
        #     rospy.loginfo('%s: Preempted' % self._action_name)
        #     self._actionServer.set_preempted()
        #     # self._result = DumpResult(False)
        # else:
        
        if(str(goal).find("startup") != -1):
            # if(goal == "startup"):
            print("startup time")
            self._result = DumpResult(True)
        elif(str(goal).find("goal1") != -1):
            self.goal1(23)
        elif(str(goal).find("goal2") != -1):
            self.goal2()

        self._actionServer.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('dump')
    print(rospy.get_name())
    server = DumpingAction(rospy.get_name())
    rospy.spin()
