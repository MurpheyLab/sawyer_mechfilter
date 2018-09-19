/*
Kathleen Fitzsimons

This node runs a timer that uses the current state of the cursor to drive the trep simulation. 
The position of the pendulum is also published.

SUBSCRIBERS:
    - cursor_state (nact3d/cursor)

PUBLISHERS:
    - mass_point (PointStamped)
    - visualization_marker_array (MarkerArray)
    - cursor_bias (Float32MultiArray)
    - cursor_dyn (nact3d/cursor_dyn)
    - trep_sys (mda_act3d/mdasys)
    - act3d_read (Bool)
    - acceptance (Bool)

SERVICES:
*/
#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include "geometry_msgs/Pose.h"
#include "intera_core_msgs/InteractionControlCommand.h"

//from intera_motion_interface import InteractionOptions
//from intera_motion_interface.utility_functions import int2bool

//#include "intera_interface"
//from intera_interface import CHECK_VERSION


class ImpedeSimulator{
    ros::Time t0 = ros::Time::now();
    ros::Duration tcurr=ros::Duration(0);
  public:
    
    ImpedeSimulator(ros::NodeHandle &nh){
      ROS_DEBUG("Creating ImpedeSimulator class");                    
    }
    //setup publishers
    ros::publisher interactCommand = n.advertise<InteractionControlCommand>(/robot/limb/right/interaction_control_command,5);
    //set up timer callback fxn
    void timercall(const ros::TimerEvent& event){
      tcurr = ros::Time::now() - t0;
      std::cout<<tcurr.toSec()<<" ";
      };
    //setup publishers, subscribers callback:
        
};
    
int main(int argc, char **argv){
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  std::cout<<"here";
  ros::init(argc, argv, "impede_test");
  ros::NodeHandle n;
  ImpedeSimulator sim(n);
  ros::Timer timer = n.createTimer(ros::Duration(0.1), &ImpedeSimulator::timercall, &sim);
  ros::spin();
 return 0;
};