#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from opencv_apps.msg import RotatedRectStamped
from geometry_msgs.msg import Twist

class track_box_to_cmd_vel:

    rect = None  ## メンバ変数として定義
    pub = None
    
    def __init__(self):
        self.rect = RotatedRectStamped()
        rospy.init_node('client')
        rospy.Subscriber('/camshift/track_box', RotatedRectStamped, self.cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist)
        rospy.Timer(rospy.Duration(0.1), self.loopOnce)
        
    def cb(self, msg):
        ## 画像処理の結果を取得
        area = msg.rect.size.width * msg.rect.size.height
        rospy.loginfo("area = {}, center = ({}, {})".format(area, msg.rect.center.x, msg.rect.center.y))
        ## 認識結果面積が一定値以上のときはrectに登録
        if area > 100 * 100:
            self.rect = msg

    def loopOnce(self, event):
        cmd_vel = Twist()
        ## 古いrect = 認識結果は利用しない
        rect_arrived = rospy.Time.now() - self.rect.header.stamp
        ## 最大1秒前の認識結果を利用
        if rect_arrived.to_sec() < 1.0:
            ## 認識結果の領域の中心のx座標が320より小さければ（画像の半分より左），左回転する
            if self.rect.rect.center.x < 320:
                cmd_vel.angular.z = 0.1
            else:
                cmd_vel.angular.z =-0.1
        ## cmd_velをpublish, 'rect_arrived.to_sec() < 1.0' がTrueにならなければ，
        ## 全ての値が０になっているcmd_velがpublishされる．つまり，ロボットは停止する．
        ## 対象物がカメラから消えた場合はロボットは止まる．
        ## こうしないと，対象物がカメラから消えても最後に送った指令が送り続けられる．
        rospy.loginfo("\t\t\t\t\t\tpublish {}".format(cmd_vel.angular.z))
        self.pub.publish(cmd_vel)
       
if __name__ == '__main__': # メイン文．ココでやっていることはtrack_box_to_cmd_vel()を呼ぶだけ．
    try:
        obj = track_box_to_cmd_vel()
        rospy.spin()
    except rospy.ROSInterruptException: pass # エラーハンドリング



