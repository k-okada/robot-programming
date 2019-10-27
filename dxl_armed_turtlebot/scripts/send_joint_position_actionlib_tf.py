#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import numpy
import actionlib
import copy
import tf
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Twist

def send_joint_position_actionlib_tf():
    # rosnode の初期化
    rospy.init_node('send_joint_position')

    # TF Listener を定義
    listener = tf.TransformListener()

    # cmd_vel パブリッシャ
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # トピック名，メッセージ型を使ってActionLib clientを定義
    client = actionlib.SimpleActionClient('/fullbody_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server() # ActionLibのサーバと通信が接続されることを確認

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # 対象物までの距離を計算する．
        dist = 1000
        try:
            # ropys.Time(0)は簡単にしたコード，最新のデータでない場合もあるので注意
            (trans,rot) = listener.lookupTransform('/base_link', '/test_object', rospy.Time(0))
            dist = numpy.linalg.norm(trans) ## 数値計算にはnumpy を利用する．
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rospy.loginfo("base_link to test_object {}".format(dist))

        if dist == 1000: # 見えていないので回転
            cmd_vel = Twist()
            cmd_vel.angular.z = 0.1
            rospy.loginfo("publish cmd_vel {}".format(cmd_vel))
            pub.publish(cmd_vel)
        elif dist > 0.6: # 近づいていく
            cmd_vel = Twist()
            cmd_vel.angular.z = trans[1]/2
            cmd_vel.linear.x = 0.05
            rospy.loginfo("publish cmd_vel {}".format(cmd_vel))            
            pub.publish(cmd_vel)
        else:
            # ActionLib client の goal を指定
            # http://wiki.ros.org/actionlib_tutorials/Tutorials の Writing a Simple Action Client (Python) を参照
            # __TOPInC_PREFIX__Action で actionlib.SimpleActionClient を初期化
            # actionlib.SimpleActionClient は __TOPIC_PREFIX__Goal を使ってゴールオブジェクトを生成
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.header.stamp = rospy.Time.now()
            goal.trajectory.joint_names = ['arm_joint1', 'arm_joint2', 'arm_joint3',
                                           'arm_joint4', 'arm_joint5', 'arm_joint6']
            point = JointTrajectoryPoint()
            point.positions = [0, 0, -math.pi/4, 0, math.pi/4, math.pi/2]
            point.time_from_start = rospy.Duration(2.0)
            goal.trajectory.points.append(point)

            # 同じデータをtrajectory.pointsに追加．copyをしないと上のtime_from_startが上書きされる
            point = copy.deepcopy(point)
            point.time_from_start = rospy.Duration(4.0)
            goal.trajectory.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [math.pi/2, 0, math.pi/4, 0, math.pi/2, math.pi/2]
            point.time_from_start = rospy.Duration(6.0)
            goal.trajectory.points.append(point)

            rospy.loginfo("send goal {}".format(goal))
            ## 目標姿勢をゴールとして送信
            client.send_goal(goal)
            rospy.loginfo("wait for goal ...")
            client.wait_for_result() # ロボットの動作が終わるまで待つ．
            rospy.loginfo("done")
            break # while 文から抜ける
        rate.sleep()

if __name__ == '__main__': # メイン文．
    try:
        send_joint_position_actionlib_tf()
    except rospy.ROSInterruptException: pass # エラーハンドリング
