#! /usr/bin/env python
import math
# import ros
import rospy
from std_msgs.msg import String
from sensor_msgs.msgs import JointState
from tf.transformations import TransformBroadcaster, quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped

# Example of importing action messages
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
# from moony_autonomy_dump.msg import DumpAction, DumpGoal, DumpResult, DumpFeedback

# Example of importing service messges
# for make a plan service
# from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse

# Example of importing general messages
# from geometry_msgs.msg import TransformStamped
# from nav_msgs.msg import Odometry, Path

# Another way to import messages
# import geometry_msgs.msg # imports 'all' messsages in this; need to access them via geometry_msgs.msg.Path

class point(object):
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __str__(self):
        return str(self.x) + ',' + str(self.y) + ',' + str(self.yaw)

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
    rospy.init_node('moony_state_publisher')
    print(rospy.get_name())
    server = DumpingAction(rospy.get_name())
    rospy.spin()

    pubJoints = rospy.Publisher('joint_states', JointState, queue_size=5)
    myBroadcaster = TransformBroadcaster()
    rate = rospy.Rate(30)  # 30 hz

    degree = 6.28/180;

    # // robot state
    tilt = 0
    tinc = degree
    swivel=0
    angle=0
    height=0
    hinc=0.005;

    # // message declarations
    odom_trans = TransformStamped()
    joint_state = JointState()
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "axis"

    while not rospy.is_shutdown():
        # //update joint_state
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name.resize(3)
        joint_state.position.resize(3)
        joint_state.name[0] ="swivel"
        joint_state.position[0] = swivel
        joint_state.name[1] ="tilt"
        joint_state.position[1] = tilt
        joint_state.name[2] ="periscope"
        joint_state.position[2] = height


        # // update transform
        # // (moving in a circle with radius=2)
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.transform.translation.x = math.cos(angle)*2
        odom_trans.transform.translation.y = math.sin(angle)*2
        odom_trans.transform.translation.z = .7
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        # //send the joint state and transform
        pubJoints.publish(joint_state)
        myBroadcaster.sendTransform(odom_trans)

        # // Create new robot state
        tilt += tinc
        if (tilt<-.5 or tilt>0): tinc *= -1
        height += hinc;
        if (height>.2 or height<0): hinc *= -1
        swivel += degree;
        angle += degree/4;

        # // This will adjust as needed per iteration
        loop_rate.sleep();