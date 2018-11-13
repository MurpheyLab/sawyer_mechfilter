#!/usr/bin/env python
import numpy as np
import copy
import rospy
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg as VM
from geometry_msgs.msg import PointStamped
from sawyer_humcpp.msg import mdasys

SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
CARTFRAME = "cart"

class Visualizer_CP:
    def __init__(self):
        rospy.loginfo("Initializing Rviz")
        #setup publisher, subscribers,timers:
        self.mda_sub = rospy.Subscriber("mda_topic",mdasys,self.update_marks)
        self.marker_pub = rospy.Publisher("visualization_marker_array", VM.MarkerArray,queue_size=3)
        #self.br = tf.TransformBroadcaster()
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
        # link marker
        self.link_marker = copy.deepcopy(mass_marker)
        self.link_marker.type = VM.Marker.LINE_STRIP
        self.link_marker.color = ColorRGBA(*[1.0, 0.05, 0.05, 1.0])#0.1, 0.1, 0.,1.0])
        self.link_marker.scale = GM.Vector3(*[0.02, 0.05, 0.05])
        self.link_marker.id = 1
        #cart marker
        self.cart_marker = copy.deepcopy(mass_marker)
        self.cart_marker.type = VM.Marker.CUBE
        self.cart_marker.color = ColorRGBA(*[0.1, 0.5, 1.0, 0.9])
        self.cart_marker.scale = GM.Vector3(*[0.2, 0.2, 0.2])
        self.cart_marker.id = 2
        
        self.markers.markers.append(mass_marker)
        self.markers.markers.append(link_marker)
        self.markers.markers.append(cart_marker)
    def update_marks(self,data):
        rospy.loginfo("got data")
        
def main():
    rospy.init_node('visualizer',log_level=rospy.info
                    
    try:
        disp=Visualizer_CP()
    except rospy.ROsInterruptException: pass
                    
if __name__'__main__':
    main()
                  
        
        
    