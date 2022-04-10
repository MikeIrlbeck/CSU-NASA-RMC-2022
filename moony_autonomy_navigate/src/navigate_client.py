#! /usr/bin/env python
import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

from moony_autonomy_navigate.msg import NavigationAction, NavigationGoal, NavigationResult, NavigationFeedback

global client
debug = False


def feedbackCb(feedback):
    print(feedback)


def navigate_client(requestedGoal):
    goal = NavigationGoal(requestedGoal)
    if debug:
        print('send goal')
    client.send_goal(goal, None, feedback_cb=feedbackCb)
    if debug:
        print('wait for result')
    client.wait_for_result()
    if debug:
        print('get result')
    return client.get_result()  # A NavigationResult

    # time.sleep(3.0)
    # client.cancel_goal()  # would cancel the goal 3 seconds after starting
    # status = client.get_state()
if __name__ == '__main__':
    rospy.init_node('navigate_client_py')
    try:
        client = actionlib.SimpleActionClient('navigate', NavigationAction)
        client.wait_for_server()

        print("Startup: " + "success" if navigate_client("startup") else "fail")
        print("in between")
        # TODO: when the line above is run, why does the goal below get passed
        print("Startup: " + "success" if navigate_client("mine") else "fail")
        print("Startup: " + "success" if navigate_client("home") else "fail")

    except rospy.ROSInterruptException:
        print("program interrupted before completion")  # , file=sys.stderr)
