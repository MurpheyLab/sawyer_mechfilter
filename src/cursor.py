#!/usr/bin/env python
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

SIMFRAME = "trep_world"
MASSFRAME = "cursor"

DT = 1./60.

class Visualizer_Cursor:
    def __init__(self):
        rospy.loginfo("Initializing Rviz")
        #setup publisher, subscribers,timers:
        self.mda_sub = rospy.Subscriber("mda_topic",mdasys,self.update_marks)
        self.marker_pub = rospy.Publisher("visualization_marker_array", VM.MarkerArray,queue_size=3)
        self.br = tf.TransformBroadcaster()
        ######SETUP MARKER ARRAY######
        self.markers = VM.MarkerArray()
         # mass marker
        self.mass_marker = VM.Marker()
        self.mass_marker.action = VM.Marker.ADD
        self.mass_marker.color = ColorRGBA(*[1.0, 1.0, 1.0, 1.0])
        self.mass_marker.header.frame_id = rospy.get_namespace() + SIMFRAME 
        self.mass_marker.lifetime = rospy.Duration(5*DT)
        self.mass_marker.scale = GM.Vector3(*[0.13, 0.13, 0.13])
        self.mass_marker.type = VM.Marker.SPHERE
        self.mass_marker.id = 0
        
        self.markers.markers.append(self.mass_marker)
        
    def update_marks(self,data):
        ptransc = [data.ef[0],data.ef[1],data.ef[2]]#data.ef[1] is the cart coord
        p = PointStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = SIMFRAME
        for m in self.markers.markers:
            m.header.stamp = p.header.stamp
        self.mass_marker.pose = GM.Pose(position=GM.Point(*ptransc))
        self.marker_pub.publish(self.markers)
                
def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('cursor', log_level=rospy.INFO)

    try:
        sim = Visualizer_Cursor()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()