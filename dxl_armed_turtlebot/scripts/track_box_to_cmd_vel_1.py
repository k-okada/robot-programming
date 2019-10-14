#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from opencv_apps.msg import RotatedRectStamped
from geometry_msgs.msg import Twist

rect = RotatedRectStamped() ## 大域変数として定義
def cb(msg):
    global rect  ## 大域変数の利用を宣言
    ## 画像処理の結果を取得
    area = msg.rect.size.width * msg.rect.size.height
    rospy.loginfo("area = {}, center = ({}, {})".format(area, msg.rect.center.x, msg.rect.center.y))
    ## 認識結果面積が一定値以上のときはrectに登録
    if area > 100 * 100:
        rect = msg

if __name__ == '__main__': # メイン文
    try:
        rospy.init_node('client')
        rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
        pub = rospy.Publisher('/cmd_vel', Twist)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ## 
            cmd_vel = Twist()
            ## 古いrect = 認識結果は利用しない
            rect_arrived = rospy.Time.now() - rect.header.stamp
            ## 最大1秒前の認識結果を利用
            if rect_arrived.to_sec() < 1.0:
                ## 認識結果の領域の中心のx座標が320より小さければ（画像の半分より左），左回転する
                if rect.rect.center.x < 320:
                    cmd_vel.angular.z = 0.1
                else:
                    cmd_vel.angular.z =-0.1
            ## cmd_velをpublish, 'rect_arrived.to_sec() < 1.0' がTrueにならなければ，
            ## 全ての値が０になっているcmd_velがpublishされる．つまり，ロボットは停止する．
            ## 対象物がカメラから消えた場合はロボットは止まる．
            ## こうしないと，対象物がカメラから消えても最後に送った指令が送り続けられる．
            rospy.loginfo("\t\t\t\t\t\tpublish {}".format(cmd_vel.angular.z))
            pub.publish(cmd_vel)
            rate.sleep()

    except rospy.ROSInterruptException: pass # エラーハンドリング



