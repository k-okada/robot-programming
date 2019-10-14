#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def send_joint_position_actionlib():
    # rosnode の初期化
    rospy.init_node('send_joint_position')
    # トピック名，メッセージ型を使ってActionLib clientを定義
    client = actionlib.SimpleActionClient('/fullbody_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server() # ActionLibのサーバと通信が接続されることを確認

    # ActionLib client の goal を指定
    # http://wiki.ros.org/actionlib_tutorials/Tutorials の Writing a Simple Action Client (Python) を参照
    # __TOPIC_PREFIX__Action で actionlib.SimpleActionClient を初期化
    # actionlib.SimpleActionClient は __TOPIC_PREFIX__Goal を使ってゴールオブジェクトを生成
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names = ['arm_joint1', 'arm_joint2', 'arm_joint3',
                                   'arm_joint4', 'arm_joint5', 'arm_joint6']
    for i in range(5):
        point = JointTrajectoryPoint()
        point.positions = [math.pi/2, 0, math.pi/4*(i%2), 0, math.pi/2, math.pi/2]
        point.time_from_start = rospy.Duration(1.0+i)
        goal.trajectory.points.append(point)

    ## 目標姿勢をゴールとして送信
    client.send_goal(goal)
    rospy.loginfo("wait for goal ...")
    client.wait_for_result() # ロボットの動作が終わるまで待つ．
    rospy.loginfo("done")

if __name__ == '__main__': # メイン文．
    try:
        send_joint_position_actionlib()
    except rospy.ROSInterruptException: pass # エラーハンドリング
