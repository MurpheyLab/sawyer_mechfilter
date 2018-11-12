#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//#include "std_msgs/Float32MultiArray.h"
//#include <sstream>
//#include <iostream>
//#include <armadillo>
//#include <random>

class VisualCP{
  ros::Subscriber mda_sub;
  ros::Publisher marker_pub;
  ros::NodeHandle* nh;  
  public:
    VisualCP(ros::NodeHandle* _nh){
    nh=_nh;
    ROS_INFO("Starting RVIZ");
    //setup publishers & subscribers
    //marker_pub = nh->advertise<sawyer_humcpp::mdasys>("mda_topic", 5);  
    //interactCommand = nh->advertise<intera_core_msgs::InteractionControlCommand>("/robot/limb/right/interaction_control_command",1);
    //cursor_state = nh->subscribe("/robot/limb/right/endpoint_state",5,&ImpedeSimulator::update_state,this);
    //end_acc = nh->subscribe("/robot/accelerometer/right_accelerometer/state",5,&ImpedeSimulator::calc_input,this);
    };
    void timercall(const ros::TimerEvent& event){
    };
    
};

int main(int argc, char **argv){
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle n;
  VisualCP marks(&n);
  ros::Timer timer = n.createTimer(ros::Duration(1./60.), &VisualCP::timercall, &marks);
  ros::spin();
 return 0;
};