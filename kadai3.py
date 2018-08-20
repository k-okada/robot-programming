#!/usr/bin/env python

#For some encoding-problem, I had to let myself go out of Japanese.

#Rospy, its looks like some kind of API. Implemented with python.
#Subscriber fetches topics, Publisher send topics.
import rospy

#Header header, RotatedRect rect ... some topics to send.
#Same for the other imported Scripts.
from oepncv_apps.msg import RotateRectStamped 
from image_view2.msg import ImageMarker2
from geometry_msgs.msg import Point

#CallBack Funcfion
def cb(msg):
    print msg.rect          #msg is expected to be rectangle object
    marker = ImageMarker2() #constructing object
    marker.type = 0         #set marker's shape into circle
    marker.position = Point(msg.rect.center.x, msg.rect.center.y, 0) 
                            #set marker's position 
    pub.publish(marker)     #send marker

rospy.init_node('client') #specified name 'client'
rospy.Subscriber('/camshift/track_box', RotatedRectStamped, cb
pub = rospy.Publisher('/image_marker', ImageMarker2)
                           
#keep python until node dies.
rospy.spin()

