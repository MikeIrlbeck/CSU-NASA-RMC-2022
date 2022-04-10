#! /usr/bin/env python
# import sys
# print(sys.path)
import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from moony_autonomy_navigate.msg import NavigationAction, NavigationGoal, NavigationResult, NavigationFeedback
from nav_msgs.msg import Odometry, Path

from tf.transformations import quaternion_from_euler, euler_from_quaternion
# from navigation_helpers import MineLocationGenerator, point # copied these classes in to avoid import errors

# for make a plan service
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from geometry_msgs.msg import PoseStamped


class point(object):
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __str__(self):
        return str(self.x) + ',' + str(self.y) + ',' + str(self.yaw)


class MineLocationGenerator(object):
    class goal(object):
        def __init__(self, x, y, yaw):
            self.x = x
            self.y = y
            self.yaw = yaw

        def __str__(self):
            return str(self.x) + ', ' + str(self.y) + ', ' + str(self.yaw)

    def __init__(self, home):
        self.home = home

    def genLocations(self, distance, number):
        # self.x_array = range(0,distance,distance/20)
        self.x_array = list()
        i = 0.0
        while (i <= float(0.01 + distance)):
            self.x_array.insert(0, i)
            i += float(distance/number)

        x_neg_array = []
        for i in range(0, len(self.x_array)):
            x_neg_array.insert(0, -1.0 * self.x_array[i])
        # print(x_neg_array)
        self.x_array.extend(x_neg_array)
        # print(self.x_array)
        self.goalList = list()
        negativeYGoalList = []
        for i in self.x_array:
            # the positive value
            y = math.sqrt(abs(math.pow(distance, 2) - i**2))
            yNeg = -1.0 * y
            yaw = math.atan2(y, i)
            yawNeg = math.atan2(yNeg, i)

            currentGoal = self.goal(self.home.x + i, self.home.y + y, yaw)
            negYGoal = self.goal(self.home.x + i, self.home.y + yNeg, yawNeg)
            self.goalList.append(currentGoal)
            negativeYGoalList.insert(0, negYGoal)
            # print(i)
        self.goalList.extend(negativeYGoalList)

    def __str__(self):
        returnString = ''
        for i in self.goalList:
            returnString += str(i) + '\n'
        return returnString


class NavigateAction(object):
    def __init__(self, name):
        self.startUp()
        self._action_name = name
        self._actionServer = actionlib.SimpleActionServer(
            self._action_name, NavigationAction, execute_cb=self.execute_cb, auto_start=False)
        self._actionServer.start()

    def initializeLocationGoal(self):
        # create messages that are used to publish feedback/result
        self._feedback = NavigationFeedback()
        self._result = NavigationResult()
        self.frame = "skid_drive"  # ensure this matches the move base package's global frame

        self.locationGoal = MoveBaseGoal()
        # TODO: see if this can be skid
        self.locationGoal.target_pose.header.frame_id = 'map'
        self.locationGoal.target_pose.pose.position.x = 0
        self.locationGoal.target_pose.pose.position.y = 0
        self.locationGoal.target_pose.pose.position.z = 0.0
        self.locationGoal.target_pose.pose.orientation.x = 0.0
        self.locationGoal.target_pose.pose.orientation.y = 0.0
        self.locationGoal.target_pose.pose.orientation.z = 0.0
        self.locationGoal.target_pose.pose.orientation.w = 1.0

        self.MoveBaseClient = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)
        self.MoveBaseClient.wait_for_server()

    def startUp(self):
        self.initializeLocationGoal()
        self.homeLocation = rospy.wait_for_message("odometry/filtered_map", Odometry, timeout=None)
        self.currentLocation = self.homeLocation
        # print(self.homeLocation)
        print("Server ACTIVE")
        # self.rotateInPlace() # this can be interrupted by the client

    def sendNavigation(self, x, y, yaw):
        quaternion = quaternion_from_euler(0, 0, yaw)
        # print("The quaternion representation is %s %s %s %s." %(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        self.locationGoal.target_pose.pose.position.x = x
        self.locationGoal.target_pose.pose.position.y = y
        self.locationGoal.target_pose.pose.orientation.z = quaternion[2]
        self.locationGoal.target_pose.pose.orientation.w = quaternion[3]
        self.MoveBaseClient.send_goal(self.locationGoal, feedback_cb=self.feedback_movebase)
        self.MoveBaseClient.wait_for_result()
        return self.MoveBaseClient.get_result()

    def feedback_movebase(self, feedback):
        print('[Feedback] Going to Goal Pose...')
        print(feedback)

    def rotateInPlace(self):
        yaw = 0.0
        while (yaw <= math.pi * 2):
            self.sendNavigation(0, 0, yaw)
            yawPer = yaw / (math.pi * 2)
            fb = NavigationFeedback("rotate in place: %5.2f%%" % (yawPer))
            self._actionServer.publish_feedback(fb)
            yaw += math.radians(135.0)
        fb = NavigationFeedback("rotate in place: 100%")
        self._actionServer.publish_feedback(fb)

    def odometry_to_point(self, location):
        quat = (location.pose.orientation.x, location.pose.orientation.y,
                location.pose.orientation.z, location.pose.orientation.w)
        euler = euler_from_quaternion(quat)
        return point(location.pose.position.x, location.pose.position.y, euler[2])

    def point_to_PoseStamped(self, locPoint):
        quaternion = quaternion_from_euler(
            locPoint.x, locPoint.y, locPoint.yaw)
        # print("locPoint quaternion  is %s %s %s %s." % (quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        thisPose = PoseStamped()
        thisPose.pose.position.x = locPoint.x
        thisPose.pose.position.y = locPoint.y
        thisPose.pose.orientation.z = quaternion[2]
        thisPose.pose.orientation.w = quaternion[3]

        # thisPose.header.seq = 0
        # thisPose.header.stamp = rospy.Time(0)
        thisPose.header.frame_id = self.frame
        return thisPose

    def check_one_goal(self, potentialGoal):
        makePlanServiceHandle = rospy.ServiceProxy(
            'move_base/make_plan', GetPlan)

        robotStart = self.point_to_PoseStamped(
            self.odometry_to_point(self.currentLocation.pose))
        robotGoal = self.point_to_PoseStamped(potentialGoal)

        makePlanSrvReq = GetPlanRequest()
        makePlanSrvReq.start = robotStart
        makePlanSrvReq.goal = robotGoal
        makePlanSrvReq.tolerance = 1.5

        # print("loc: %.2f, %.2f -> goal: %.2f, %.2f" % (makePlanSrvReq.start.pose.position.x, makePlanSrvReq.start.pose.position.y, makePlanSrvReq.goal.pose.position.x, makePlanSrvReq.goal.pose.position.y))

        pathPlanResponse = makePlanServiceHandle.call(makePlanSrvReq)
        myPath = pathPlanResponse.plan
        myPath.header.frame_id = self.frame

        pub = rospy.Publisher('chatter', Path, queue_size=100)
        i = 0.0
        rate = rospy.Rate(10)  # 10 hz
        while (i < 3):
            pub.publish(myPath)
            rate.sleep()
            i += 1
        return len(myPath.poses)

    def findGoalIndex(self, goalTuple):
        if (len(self._orderedGoalList) == 0):
            return 0
        for i in range(0, len(self._orderedGoalList)):
            if (goalTuple[0] < (self._orderedGoalList[i])[0]):
                return i
        return len(self._orderedGoalList)

    def insertAndOrderGoals(self, goalTuple):
        goalIndex = self.findGoalIndex(goalTuple)
        self._orderedGoalList.insert(goalIndex, goalTuple)

    def check_goals(self, goalList):
        # update current location
        self.currentLocation = rospy.wait_for_message("odometry/filtered_map", Odometry, timeout=None)
        rospy.wait_for_service('move_base/make_plan')

        self._orderedGoalList = []

        print("original ordering of goals")
        for i in goalList:
            goalCost = self.check_one_goal(i)
            myGoalTuple = (goalCost, i)
            # print("cost %d, goal: %s" % (myGoalTuple[0], myGoalTuple[1]))
            self.insertAndOrderGoals(myGoalTuple)

    def go_mine(self, distance):
        homePoint = self.odometry_to_point(self.homeLocation.pose)
        print("My home location: %s" % homePoint)
        myGen = MineLocationGenerator(homePoint)
        myGen.genLocations(distance, 5)
        # self.goalCost = len(myGen.goalList)
        # print(self.goalCost)

        # bestGoal =
        self.check_goals(myGen.goalList)
        print("printing goals that have been ordered:\n")
        for i in self._orderedGoalList:
            print("cost %d, goal: %s" % (i[0], i[1]))
            result = self.sendNavigation(i[1].x, i[1].y, i[1].yaw)
            print(result)
            break # TODO: if successful, stop we have arrived
    def go_home(self):
        homePoint = self.odometry_to_point(self.homeLocation.pose)
        result = self.sendNavigation(homePoint.x, homePoint.y, homePoint.yaw)
        print(result)
    def execute_cb(self, goal):
        # helper variables
        # r = rospy.Rate(1)
        fb = NavigationFeedback("starting")
        self._actionServer.publish_feedback(fb)

        success = True
        print(goal)
        if(str(goal).find("startup") != -1):
            # if(goal == "startup"):
            print("startup time")
            self.rotateInPlace()
        elif(str(goal).find("mine") != -1): # TODO: allow client to determine mining distance
            self.go_mine(4)
        elif(str(goal).find("home") != -1): 
            self.go_home()
        
        # Uncomment these lines to test goal preemption:
        # time.sleep(3.0)
        # client.cancel_goal()  # would cancel the goal 3 seconds after starting
        # status = client.get_state()

        # # publish info to the console for the user
        # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

#     if self._actionServer.is_preempt_requested(): # allows the client to cancel the goal
#         rospy.loginfo('%s: Preempted' % self._action_name)
#         self._actionServer.set_preempted()
#         success = False

        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     # publish the feedback
        #     self._actionServer.publish_feedback(self._feedback)

        _result = True
        if success:
            # self._result.sequence = self._feedback.sequence
            # rospy.loginfo('%s: Succeeded' % self._action_name)
            self._actionServer.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('navigate')
    print(rospy.get_name())
    server = NavigateAction(rospy.get_name())
    rospy.spin()
