#! /usr/bin/env python
import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

from moony_autonomy_navigate.msg import NavigationAction, NavigationGoal, NavigationResult, NavigationFeedback

debug = False

def navigate_client(requestedGoal):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('navigate', NavigationAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal_startup = NavigationGoal("startup")
    # Sends the goal to the action server.
    if debug:
        print('send goal')
    client.send_goal(goal_startup)

    # Waits for the server to finish performing the action.
    if debug:
        print('wait for result')
    client.wait_for_result()
    
    # # Prints out the result of executing the action
    # if debug:
    #     print('get result')
    # return client.get_result()  # A NavigationResult

    goal_mine = NavigationGoal("mine")

    # Sends the goal to the action server.
    if debug:
        print('send goal')
    client.send_goal(goal_mine) # HERE ADD doneCb, activeCb, and feedback Cb

    # Waits for the server to finish performing the action.
    if debug:
        print('wait for result')
    client.wait_for_result()
    
    # Prints out the result of executing the action
    if debug:
        print('get result')
    return client.get_result()  # A NavigationResult

    #time.sleep(3.0)
    #client.cancel_goal()  # would cancel the goal 3 seconds after starting
    # status = client.get_state()
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('navigate_client_py')
        result = navigate_client()
        print("Result: " + "success" if result else "fail")
    except rospy.ROSInterruptException:
        print("program interrupted before completion")#, file=sys.stderr)