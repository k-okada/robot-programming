#!/usr/bin/env roseus

(ros::load-ros-package "roseus")
(ros::load-ros-package "opencv_apps")
(ros::load-ros-package "geometry_msgs")

(setq rect (instance opencv_apps::RotatedRectStamped :init)) ;; 大域変数として定義

;; def cb(msg):
(defun cb (msg)
  ;; 画像処理の結果を取得
  ;; area = msg.rect.size.width * msg.rect.size.height
  (setq area (* (send msg :rect :size :width) (send msg :rect :size :height)))
  ;; rospy.loginfo("area = {}, center = ({}, {})".format(area, msg.rect.center.x, msg.rect.center.y))
  (ros::ros-info "area = ~A, center = (~A, ~A)" area (send msg :rect :center :x) (send msg :rect :center :y))
  ;; 認識結果面積が一定値以上のときはrectに登録
  (if (> area (* 100 100))
      (setq rect msg)))

;; ノードの初期化
;; rospy.init_node('client')
;; rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
;; pub = rospy.Publisher('/cmd_vel', Twist)
;; rate = rospy.Rate(10)
(ros::roseus "client")
(ros::subscribe "/camshift/track_box" opencv_apps::RotatedRectStamped #'cb)
(ros::advertise "/cmd_vel" geometry_msgs::Twist 1)
(ros::rate 10)

;; while not rospy.is_shutdown():
;; 対応するroseusコードは(while (not (ros::ok))
;; do-until-keyでC-c以外でもループから抜けるようにした．
(do-until-key
 ;; メッセージのインスタンスの生成
 ;; cmd_vel = Twist()
 (setq cmd_vel (instance geometry_msgs::Twist :init))
 ;; 古いrect = 認識結果は利用しない
 ;; rect_arrived = rospy.Time.now() - rect.header.stamp
 (setq rect_arrived (ros::time- (ros::time-now) (send rect :header :stamp)))
 ;;最大1秒前の認識結果を利用
 ;; if rect_arrived.to_sec() < 1.0:
 (when (< (send rect_arrived :to-sec) 1.0)
   ;;## 認識結果の領域の中心のx座標が320より小さければ（画像の半分より左），左回転する
   (if (< (send rect :rect :center :x) 320)
       (send cmd_vel :angular :z 0.1)
     (send cmd_vel :angular :z -0.1))
   ;; rospy.loginfo("\t\t\t\t\t\tpublish {}".format(cmd_vel.angular.z))
   (ros::ros-info "~t~t~t~t~tpublish ~A" (send cmd_vel :angular :z))
   ;; pub.publish(cmd_vel)
   (ros::publish "/cmd_vel" cmd_vel)
   ;; rate.sleep()
   (ros::sleep)
   ) ;; when (< のループの終了
 ;; roseusではこれが必要．
 (ros::spin-once)
 ) ;; do-until-key のループの終了
