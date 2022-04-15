#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

from moony_autonomy_mine.msg import MineAction, MineGoal, MineResult, MineFeedback

# global client

def feedbackCb(feedback):
    print(feedback)

def Mining_client(requestedGoal):
    goal = MineGoal(requestedGoal)
    client.send_goal(goal, None, feedback_cb=feedbackCb)

    # Other client functionality
    # client.cancel_goal()  # would cancel the goal 3 seconds after starting
    # print(client.get_state())
    
    client.wait_for_result()
    return client.get_result()  # A MineResult
if __name__ == '__main__':
    rospy.init_node('mine_client_py')
    try:
        client = actionlib.SimpleActionClient('mine', MineAction)
        client.wait_for_server()

        print(Mining_client("startup"))
        print(Mining_client("goal1"))
        print(Mining_client("goal2"))

    except rospy.ROSInterruptException:
        print("program interrupted before completion") 
