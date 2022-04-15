#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

from moony_autonomy_dump.msg import DumpAction, DumpGoal, DumpResult, DumpFeedback

# global client

def feedbackCb(feedback):
    print(feedback)

def dumping_client(requestedGoal):
    goal = DumpGoal(requestedGoal)
    client.send_goal(goal, None, feedback_cb=feedbackCb)

    # Other client functionality
    # client.cancel_goal()  # would cancel the goal 3 seconds after starting
    # print(client.get_state())
    
    client.wait_for_result()
    return client.get_result()  # A DumpResult
if __name__ == '__main__':
    rospy.init_node('dump_client_py')
    try:
        client = actionlib.SimpleActionClient('dump', DumpAction)
        client.wait_for_server()

        print(dumping_client("startup"))
        print(dumping_client("goal1"))
        print(dumping_client("goal2"))

    except rospy.ROSInterruptException:
        print("program interrupted before completion") 
