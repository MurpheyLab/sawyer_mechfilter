#!/usr/bin/env python
"""
Kathleen Fitzsimons

This node sets up a marker array for visualizing the current position of 
the end effector as well as a points array for visualizing the trajectory
history of the end effector. The Marker array is published to Rviz. Markers
are updated using a callback function that takes the custom data collection
message as its argument.

SUBSCRIBERS:
    - mda_sub (mda_topic)
    
PUBLISHERS:
    - marker_pub (visualization_marker_array)
 
SERVICES:
"""

import numpy as np
import copy
import rospy
from std_msgs.msg import ColorRGBA
import geometry_msgs.msg as GM
import visualization_msgs.msg as VM
import tf
from tf import transformations as TR
from geometry_msgs.msg import PointStamped
from sawyer_humcpp.msg import mdasys
from std_msgs.msg import Bool
import time

SIMFRAME = "trep_world"
MASSFRAME = "cursor"

DT = 1./100.
T=10.
class Visualizer_Cursor:
    def __init__(self):
        rospy.loginfo("Initializing Rviz")
        #setup publisher, subscribers,timers:
        self.mda_sub = rospy.Subscriber("mda_topic",mdasys,self.update_marks)
        self.marker_pub = rospy.Publisher("visualization_marker_array", VM.MarkerArray,queue_size=3)
        self.trial_pub = rospy.Publisher("trial_topic", Bool,queue_size=1)
        self.br = tf.TransformBroadcaster()
        ######SETUP MARKER ARRAY######
        self.markers = VM.MarkerArray()
         # mass marker
        self.mass_marker = VM.Marker()
        self.mass_marker.action = VM.Marker.ADD
        self.mass_marker.color = ColorRGBA(*[1.0, 1.0, 1.0, 1.0])
        self.mass_marker.header.frame_id = rospy.get_namespace() + SIMFRAME 
        self.mass_marker.lifetime = rospy.Duration(5*DT)
        self.mass_marker.scale = GM.Vector3(*[0.05, 0.05, 0.05])
        self.mass_marker.type = VM.Marker.SPHERE
        self.mass_marker.id = 0
        
        self.draw_marker = VM.Marker()
        self.draw_marker.action = VM.Marker.ADD
        self.draw_marker.color = ColorRGBA(*[1.0, 1.0, 1.0, 1.0])
        self.draw_marker.header.frame_id = rospy.get_namespace() + SIMFRAME 
        self.draw_marker.lifetime = rospy.Duration(100*DT)
        self.draw_marker.scale = GM.Vector3(*[0.01, 0.01, 0.01])
        self.draw_marker.type = VM.Marker.POINTS
        self.draw_marker.id = 1
        
        self.bound_marker = VM.Marker()
        self.bound_marker.action
        self.bound_marker.action = VM.Marker.ADD
        self.bound_marker.color = ColorRGBA(*[1.0, 0.1, 0.1, 1.0])
        self.bound_marker.header.frame_id = rospy.get_namespace() + SIMFRAME 
        self.bound_marker.lifetime = rospy.Duration(100*DT)
        self.bound_marker.scale = GM.Vector3(*[0.01, 0.01, 0.01])
        self.bound_marker.type = VM.Marker.LINE_STRIP
        pz = 0.2
        p1 = [0.3,0.4,0.2]
        p2 = [0.9,0.4,0.2]
        p3 = [0.9,-0.2,0.2]
        p4 = [0.3,-0.2,0.2]
        self.bound_marker.points = [GM.Point(*p1),GM.Point(*p2),GM.Point(*p3),GM.Point(*p4),GM.Point(*p1)]
        self.bound_marker.id = 2

        self.timer_marker = VM.Marker()
        self.timer_marker.action = VM.Marker.ADD
        self.timer_marker.color = ColorRGBA(*[1.0, 0.05, 0.05, 1.0])
        self.timer_marker.header.frame_id = rospy.get_namespace() + SIMFRAME 
        self.timer_marker.lifetime = rospy.Duration(100*DT)
        self.timer_marker.scale = GM.Vector3(*[0.01, 0.01, 0.01])
        self.timer_marker.type = VM.Marker.TEXT_VIEW_FACING
        self.timer_marker.scale = GM.Vector3(*[0.1, 0.1, 0.1])
        self.timer_marker.pose.position.x = 0.45;
    	self.timer_marker.pose.position.y = -0.2;
    	self.timer_marker.pose.position.z = 1.0;
    	self.timer_marker.pose.orientation.x = 0.;
    	self.timer_marker.pose.orientation.y = 0.0;
    	self.timer_marker.pose.orientation.z = 0.2;
    	self.timer_marker.pose.orientation.w = 1.0;
    	self.timer_marker.text = "X"
    	self.timer_marker.id = 3
        
        self.markers.markers.append(self.mass_marker)
        self.markers.markers.append(self.draw_marker)
        self.markers.markers.append(self.bound_marker)
        self.markers.markers.append(self.timer_marker)
        
        self.testno = 0
        #set up subsampling of points
        self.subsamp = 0
        
    def update_marks(self,data):
        if data.sys_time>=T and data.sys_time<=T+DT:
            rospy.loginfo("Trial Over")
            self.testno = self.testno+1
            print "TRIAL NO",self.testno
            self.marker_pub.publish(self.markers)
            ready = False
            while ready == False:
                switch = raw_input("Is the subject ready to continue? [y/n]")
                if switch == 'y' or switch == "Y" or switch == "yes" or switch == "Yes":
                    ready = True
            time.sleep(1.0)
            self.trial_pub.publish(True)
        ptransc = [data.ef[0],data.ef[1],data.ef[2]]
        p = PointStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = SIMFRAME
        for m in self.markers.markers:
            m.header.stamp = p.header.stamp
        if self.subsamp == 0:    
            self.draw_marker.points.append(GM.Point(*ptransc))
            self.subsamp +=1
        elif self.subsamp == 4:
            self.subsamp = 0
        else:
            self.subsamp+=1
        self.mass_marker.pose = GM.Pose(position=GM.Point(*ptransc))
        self.timer_marker.text = "%3.2f" % data.sys_time
        if data.sys_time<=T:
            self.marker_pub.publish(self.markers)
                
def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('cursor_sm', log_level=rospy.INFO)

    try:
        sim = Visualizer_Cursor()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
