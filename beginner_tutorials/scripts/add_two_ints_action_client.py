#! /usr/bin/env python

import rospy

import actionlib

import beginner_tutorials.msg

def add_two_ints_client():
    client = actionlib.SimpleActionClient('add_two_ints', beginner_tutorials.msg.AddTwoIntsAction)
    client.wait_for_server()
    goal = beginner_tutorials.msg.AddTwoIntsGoal(a=20, b=30)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()  # A AddTwoIntsResult

if __name__ == '__main__':
    try:
        rospy.init_node('add_two_ints_client_py')
        result = add_two_ints_client()
        rospy.loginfo("Result: {}".format(result))
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
