/*
Kathleen Fitzsimons

This node runs a timer that uses the current state of Sawyer's end effector
to calculate a feedback force based on the end effector's distance from the
nearest black pixel in the prescribed image.
It subscribes to the endpoint_state of the robot limb and publishes an 
interaction control command. A custom message for synchronized data collection
is also published.

SUBSCRIBERS:
    - cursor_state (/robot/limb/right/endpoint_state)

PUBLISHERS:
    -mda_pub (mda_topic)
    -interact_command (/robot/limb/right/interaction_control_command)

SERVICES:N/A
*/
#include<typeinfo>
#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <intera_core_msgs/InteractionControlCommand.h>
#include <intera_core_msgs/EndpointState.h>
#include <sensor_msgs/Imu.h>

//Custom includes
#include "sawyer_humcpp/mdasys.h"
#include"Virtual_Fixt/imagewalls.hpp"

const double SCALE = 1.0;
const double DT=1./60.;
const double Kp = 1.0;
const double Kd = 50.;

using namespace std;

class Walls{
  ros::Time t0=ros::Time::now(); 
  ros::NodeHandle* nh;
  ros::Subscriber cursor_state;
  ros::Publisher interactCommand;
  ros::Publisher mda_pub;
  intera_core_msgs::InteractionControlCommand interactopt;
  sawyer_humcpp::mdasys currstate;
  tf2::Quaternion q_acc, q_ep,q_ep_force,q_ep_vel;
  bool initcon=false;
  imagewalls* apple;
      
  public:
  Walls(ros::NodeHandle* _nh, imagewalls*_wall){
    nh=_nh; apple = _wall;
    ROS_INFO("Creating Virtual Fixture class");
    //setup publishers & subscribers
    mda_pub = nh->advertise<sawyer_humcpp::mdasys>("mda_topic", 5);
    cursor_state = nh->subscribe("/robot/limb/right/endpoint_state",5,&Walls::update_state,this);
    interactCommand = nh->advertise<intera_core_msgs::InteractionControlCommand>("/robot/limb/right/interaction_control_command",1);  
    
  };
  
  //set up timer callback fxn
  void timercall(const ros::TimerEvent& event){
    if(initcon==false) {return;};
    interact_options(currstate.q[0],currstate.q[1]);
    interactCommand.publish(interactopt);
   };
  
  //state update from end effector
  void update_state(const intera_core_msgs::EndpointState& state){
    currstate.sys_time = (ros::Time::now() - t0).toSec();
    double xcurr,ycurr;
    xcurr = SCALE*state.pose.position.x-0.6; ycurr = SCALE*state.pose.position.y-0.1;//center at home position
    xcurr = (xcurr+0.3)*apple->width/0.6;//pushing x between 0 and the pixelwidth of the image
    ycurr = (ycurr+0.3)*apple->height/0.6;
    currstate.q = {xcurr,ycurr,SCALE*state.pose.position.z};
    q_ep.setValue(state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w);
    tf2::Quaternion q_vel_temp; q_vel_temp.setValue(state.twist.linear.x,state.twist.linear.y,state.twist.linear.z,0.0);
    q_ep_vel = q_ep*q_vel_temp*q_ep.inverse();
    currstate.dq = {SCALE*q_ep_vel.x(),SCALE*q_ep_vel.y()};
    tf2::Quaternion q_force_temp; q_force_temp.setValue(state.wrench.force.x,state.wrench.force.y,state.wrench.force.z,0.0);
    q_ep_force = q_ep*q_force_temp*q_ep.inverse();
    currstate.u = {q_ep_force.y(),SCALE*q_acc.y()};
    currstate.ef = {SCALE*state.pose.position.x,SCALE*state.pose.position.y,SCALE*state.pose.position.z};
    mda_pub.publish(currstate);
    if(initcon==false){initcon=true;ROS_INFO("Initcon set");};
    };
  
  //setting the impedance of the interaction options message
  void interact_options(int x, int y){
    interactopt.header.stamp = ros::Time::now();
    interactopt.header.seq=1;
    interactopt.header.frame_id = "base";
    interactopt.interaction_control_active = true;
    interactopt.interaction_control_mode = {2,2,1,1,1,1};
    interactopt.K_impedance = {0,0,500,5000,1000,1000};
    interactopt.max_impedance = {false,false,false,false,false,false};
    interactopt.D_impedance = {0,0,8.,0,2,2};
    interactopt.K_nullspace = {0.,10.,10.,0.,100.,0.,0.};
    arma::vec Fwall = apple->wallforce(x,y);
    currstate.bf = Fwall(0);
    interactopt.force_command = {Fwall(0)-Kd*currstate.dq[0],Fwall(1)-Kd*currstate.dq[1],0.,0.,0.,0.};
    if(abs(Fwall(0))>0.25 or abs(Fwall(1))>0.25){ROS_INFO("Boundary Violation");};
    interactopt.interaction_frame.position.x = 0;
    interactopt.interaction_frame.position.y =0;
    interactopt.interaction_frame.position.z  =0;
    interactopt.interaction_frame.orientation.x = 0;
    interactopt.interaction_frame.orientation.y = 0;
    interactopt.interaction_frame.orientation.z = 0;
    interactopt.interaction_frame.orientation.w = 1;
    interactopt.endpoint_name ="right_hand";
    interactopt.in_endpoint_frame = false;
    interactopt.disable_damping_in_force_control = true;
    interactopt.disable_reference_resetting = false;
    interactopt.rotations_for_constrained_zeroG = false;
  };
  
  void set_traj(){ 
         
  };
        
};
    
int main(int argc, char **argv){
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  ros::init(argc, argv, "walls");
  ros::NodeHandle n;
  string imageName("/home/kt-fitz/sawyer_ws/src/sawyer_humcpp/src/apple.png");
  imagewalls apple(imageName, 100.,Kp);
  Walls sim(&n, &apple);
  ros::Timer timer = n.createTimer(ros::Duration(DT), &Walls::timercall, &sim);
  ros::spin();
return 0;
};