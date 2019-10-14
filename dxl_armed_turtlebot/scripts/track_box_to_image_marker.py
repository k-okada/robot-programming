#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from opencv_apps.msg import RotatedRectStamped
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

def cb(msg):
    print msg.rect
    marker = ImageMarker2()
    ## http://docs.ros.org/melodic/api/image_view2/html/msg/ImageMarker2.html 
    ## または，rosmsg show -r image_view2/ImageMarker2 参照．
    ## marker.type = 0 は，CIRCLE=0 を使うことを意味している．
    marker.type = 0
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0)
    pub.publish(marker)

rospy.init_node('client')
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb)
pub = rospy.Publisher('/image_marker', ImageMarker2)
rospy.spin()
