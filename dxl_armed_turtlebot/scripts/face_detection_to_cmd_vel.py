#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from opencv_apps.msg import FaceArrayStamped
from geometry_msgs.msg import Twist

result = FaceArrayStamped() ## 大域変数として定義
def cb(msg):
    global result  ## 大域変数の利用を宣言
    result = msg
    rospy.loginfo("faces = {}".format(len(result.faces)))

if __name__ == '__main__': # メイン文
    try:
        rospy.init_node('client')
        rospy.Subscriber('/face_detection/faces', FaceArrayStamped, cb)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            if (len(result.faces)) > 0:
                face = result.faces[0]
                rospy.loginfo("faces = {}, {}".format(face.face.x, face.face.y))
                if face.face.x < 320:
                    cmd_vel.angular.z = 0.1
                else:
                    cmd_vel.angular.z =-0.1
            rospy.loginfo("\t\t\t\t\t\tpublish {}".format(cmd_vel.angular.z))
            pub.publish(cmd_vel)
            rate.sleep()

    except rospy.ROSInterruptException: pass # エラーハンドリング



