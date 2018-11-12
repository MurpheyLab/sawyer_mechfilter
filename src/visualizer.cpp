#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "sawyer_humcpp/mdasys.h"
//#include "std_msgs/Float32MultiArray.h"
//#include <sstream>
//#include <iostream>
//#include <armadillo>
//#include <random>
using namespace std;
class VisualCP{
  ros::Subscriber mda_sub;
  ros::Publisher marker_pub;
  ros::NodeHandle* nh;
  //visualization_msgs::MarkerArray markerset;
  //Mass Marker
  visualization_msgs::Marker mass;
    
   visualization_msgs::Marker marker;std::cout<<"wtf"<<std::endl;
   //marker.header.frame_id = "base_link";
   //marker.header.stamp = ros::Time();
    //mass.header.frame_id = "cart";
    /*mass.header.stamp=ros::Time();
    mass.ns = ros::this_node::getNamespace()+"sim_world";
    mass.id = 0;
    mass.type = visualization_msgs::Marker::SPHERE;
    mass.action=visualization_msgs::Marker::ADD;
    mass.scale.x=0.13 ;mass.scale.y=0.13;mass.scale.z=0.13;
    mass.color.a=1.0;mass.color.r=1.0;mass.color.g=1.0;mass.color.b=1.0;*/
  //Mass Marker
  /*visualization_msgs::Marker link;
    link.header.frame_id = "link";
    link.header.stamp=mass.header.stamp;
    link.ns = ros::this_node::getNamespace()+"sim_world";
    link.id = 1;
    link.type = visualization_msgs::Marker::LINE_STRIP;
    link.action=visualization_msgs::Marker::ADD;
    link.scale.x=0.13 ;mass.scale.y=0.13;mass.scale.z=0.13;
    link.color.a=1.0;mass.color.r=1.0;mass.color.g=1.0;mass.color.b=1.0;
  */
  
  public:
    VisualCP(ros::NodeHandle* _nh){
    nh=_nh;
    ROS_INFO("Starting RVIZ");
    //setup publishers & subscribers
    marker_pub = nh->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 5);  
    mda_sub = nh->subscribe("/mda_topic",5,&VisualCP::update_state,this);
    };
    void timercall(const ros::TimerEvent& event){
    };
    void update_state(const sawyer_humcpp::mdasys& state){
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