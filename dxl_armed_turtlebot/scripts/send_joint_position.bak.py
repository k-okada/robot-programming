#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# class GazeboSubscriberListener(rospy.SubscribeListener):
#     def __init__(self):
#         super(GazeboSubscriberListener, self).__init__()

#     def peer_subscribe(self, topic_name, topic_publish, peer_publish):
#         #rospy.loginfo("Subscribed")
#         #rospy.loginfo(topic_name)
#         rospy.loginfo(topic_publish)
#         print(dir(topic_publish))
#         rospy.loginfo(peer_publish)
#         print(dir(peer_publish))

def send_joint_position():
    # rosnode の初期化
    rospy.init_node('send_joint_position')
    # トピック名，メッセージ型を使ってパブリッシャを定義
    #pub = rospy.Publisher('/fullbody_controller/command', JointTrajectory, queue_size=1)

    ## publihser がサブスクライバと接続するのを待つ
    ## これをしないと接続する前にpub.pusblish()が呼ばれる
    ## 簡単にはrospy.sleep(1)などでもよい．
    #rospy.sleep(1)
    sl = GazeboSubscriberListener()
    pub = rospy.Publisher('/fullbody_controller/command', JointTrajectory, queue_size=1, subscriber_listener=sl)

    ##
    joint_trajectory = JointTrajectory()

    joint_trajectory.header.stamp = rospy.Time.now()
    joint_trajectory.joint_names = ['arm_joint1', 'arm_joint2', 'arm_joint3',
                                    'arm_joint4', 'arm_joint5', 'arm_joint6']
    for i in range(5):
        point = JointTrajectoryPoint()
        point.positions = [math.pi/2, 0, math.pi/4*(i%2), 0, math.pi/2, math.pi/2]
        point.time_from_start = rospy.Duration(1.0+i)
        joint_trajectory.points.append(point)

    pub.publish(joint_trajectory)
    ## 動作終了を待つ
    rospy.sleep(5)

if __name__ == '__main__': # メイン文．
    try:
        send_joint_position()
    except rospy.ROSInterruptException: pass # エラーハンドリング
