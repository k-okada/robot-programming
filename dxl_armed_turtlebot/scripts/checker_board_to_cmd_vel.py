#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import Twist

class checker_board_to_cmd_vel:
    pub = None
    
    def __init__(self):
        rospy.init_node('client')
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.loopOnce)
        
    def loopOnce(self, event):
        try:
            ## 1秒以内に得られた認識結果を利用
            tm = self.listener.getLatestCommonTime('/base_link', '/test_object')
            rospy.loginfo("get latest common time is {} before".format((rospy.Time.now() - tm).to_sec()))
            if (rospy.Time.now() - tm).to_sec() < 1.0:
                (trans,rot) = self.listener.lookupTransform('/base_link', '/test_object', tm)
                rospy.loginfo("lookup transform y = {}".format(trans[1]))
                cmd_vel = Twist()
                cmd_vel.angular.z = trans[1]
                self.pub.publish(cmd_vel)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            ## 対象物が見つからなければ１秒左に回る
            rospy.loginfo("turn around to look for target ...")
            cmd_vel = Twist()
            cmd_vel.angular.z = 0.2
            start_time = rospy.Time.now()
            # $ rostopic pub /cmd_vel geometry_msgs/Twist  '{angular: {z: -0.1}}'
            # としたらわかるように/cmd_velに一回指令を送っても回転し続けるわけではない
            while (rospy.Time.now() - start_time).to_sec() < 2.0:
                self.pub.publish(cmd_vel)
                rospy.sleep(rospy.Duration(0.1)) # http://wiki.ros.org/rospy/Overview/Time
            rospy.loginfo(" ... done")
       
if __name__ == '__main__': # メイン文．ココでやっていることはchecker_board_to_cmd_vel()を呼ぶだけ．
    try:
        obj = checker_board_to_cmd_vel()
        rospy.spin()
    except rospy.ROSInterruptException: pass # エラーハンドリング



