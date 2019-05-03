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
#include<typeinfo>
#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <intera_core_msgs/InteractionControlCommand.h>
#include <intera_core_msgs/EndpointState.h>
#include <sensor_msgs/Imu.h>
//#include <intera_interface/RobotEnable.h>
//#include<intera_interface>

//MDA Headers
#include <fstream>
#include<math.h>
#include<armadillo>
#include"SAC_cartpend/cartpend.hpp"
#include"SAC_cartpend/error_cost.hpp"
#include"SAC_cartpend/SAC.hpp"
#include"SAC_cartpend/rk4_int.hpp"
#include "sawyer_humcpp/mdasys.h"
#include"SAC_cartpend/MDA.hpp"
using namespace std;

const double DT=1./100.;
const double SCALE = 4.0;
const double Kv = 150.0;

template <class system, class objective,class sac>
class ImpedeSimulator{
  system* sys; 
  objective* cost;
  sac* sacsys;
  mda* demon;
  ros::Time t0 = ros::Time::now();
  ros::Duration tcurr=ros::Duration(0);
  ros::NodeHandle* nh;
  intera_core_msgs::InteractionControlCommand interactopt;
  ros::Publisher interactCommand;
  ros::Publisher mda_pub;
  ros::Subscriber cursor_state;
  ros::Subscriber end_acc;
  float xprev[3];
  float vd = 0.,vact=0.;
  sawyer_humcpp::mdasys currstate;
  tf2::Quaternion q_acc, q_ep,q_ep_force;
  bool initcon=false;
  
  
  public:
  
  ImpedeSimulator(ros::NodeHandle* _nh,system *_sys, objective *_cost,sac *_sacsys,mda *_demon){
      nh=_nh;sys = _sys; cost=_cost;sacsys=_sacsys;demon=_demon;
    ROS_INFO("Creating ImpedeSimulator class");
    //setup publishers & subscribers
    mda_pub = nh->advertise<sawyer_humcpp::mdasys>("mda_topic", 5);  
    interactCommand = nh->advertise<intera_core_msgs::InteractionControlCommand>("/robot/limb/right/interaction_control_command",1);
    cursor_state = nh->subscribe("/robot/limb/right/endpoint_state",5,&ImpedeSimulator::update_state,this);
    end_acc = nh->subscribe("/robot/accelerometer/right_accelerometer/state",5,&ImpedeSimulator::calc_input,this);
    
    };
  
  //set up timer callback fxn
  void timercall(const ros::TimerEvent& event){
    tcurr = ros::Time::now() - t0;
    if(initcon==false) {return;};
    if(currstate.accept){
      interact_options(false);
      vd = (xprev[2]-xprev[1])/DT;
    }//ROS_INFO("UnLocked");}
      else{interact_options(true);};
    interactCommand.publish(interactopt);
    
    //ROS_INFO("Time Now: %f",tcurr.toSec());
  };
  
  //state update from end effector
  void update_state(const intera_core_msgs::EndpointState& state){
      currstate.sys_time = (ros::Time::now() - t0).toSec();
      q_ep.setValue(state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w);
      tf2::Quaternion q_force_temp; q_force_temp.setValue(state.wrench.force.x,state.wrench.force.y,state.wrench.force.z,0.0);
      q_ep_force = q_ep*q_force_temp*q_ep.inverse();
      currstate.u = {q_ep_force.y(),SCALE*q_acc.y()};
      currstate.ef = {SCALE*(float)state.pose.position.x,SCALE*(float)state.pose.position.y,SCALE*(float)state.pose.position.z};
      if(initcon==false)
        {sys->Xcurr = {PI,0.,SCALE*state.pose.position.y,0.}; sys->Ucurr={0.0}; 
         initcon=true;ROS_INFO("Initcon set");
         xprev[0]=SCALE*state.pose.position.y;
         xprev[1]=SCALE*state.pose.position.y;
         xprev[2]=SCALE*state.pose.position.y;
      };      
      xprev[0] = xprev[1]; xprev[1]=xprev[2]; xprev[2]=SCALE*(float)state.pose.position.y;
      vact=(xprev[2]-xprev[1])/DT;
      currstate.q = {(float)sys->Xcurr[0],(float)sys->Xcurr[2]};
      arma::mat xdot = sys->f(sys->Xcurr,sys->Ucurr);
      currstate.dq = {(float)xdot(0),(float)xdot(2)};
      currstate.ddq = {(float)xdot(1),(float)xdot(3)};
      sys->Ucurr = {-(2.0*xprev[1]-xprev[2]-xprev[0])/(DT*DT)}; 
      sys->step();
      sacsys->SAC_calc();
      currstate.sac = {(float)sacsys->ulist(0)};
      arma::vec tempu = {SCALE*q_acc.y()};
      currstate.accept = demon->filter(sacsys->ulist.col(0),tempu);//sys->Ucurr);
      mda_pub.publish(currstate);
      
};
  
  void calc_input(const sensor_msgs::Imu& imu){
    tf2::Quaternion q_a_temp;
    q_a_temp.setValue(imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z,0.0);
    q_acc = q_ep*q_a_temp*q_ep.inverse();   
    
  };
  
  //setting the impedance of the interaction options message
  void interact_options(bool reject){
    interactopt.header.stamp = ros::Time::now();
    interactopt.header.seq=1;
    interactopt.header.frame_id = "base";
    interactopt.interaction_control_active = true;
    interactopt.interaction_control_mode = {1,1,1,1,1,1};
    interactopt.K_impedance = {0,0,1300,1000,1000,1000};
    interactopt.max_impedance = {true,false,true,true,true,true};
    interactopt.D_impedance = {0,0,8.,0,2,2};
    interactopt.K_nullspace = {0.,10.,10.,0.,0.,0.,0.};
    interactopt.force_command = {0.,0.,0.,0.,0.,0.};
    //if(vact<-0.1 && vd>0.1){ROS_INFO("sign issue");}
    if(reject==true){
        interactopt.D_impedance = {0,50*(vact-vd),8.,0,2,2};//without a gain this maxes out at 6
    }
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
        
};
    
arma::vec xd(double t){
    return arma::zeros(4);};
  arma::vec unom(double t){
    return arma::zeros(1);};
int main(int argc, char **argv){
  CartPend syst1{0.1,0.1,9.81,2.0,0.01};
  arma::mat Q = {
    {200,0.,0.,0.},
    {0., 0.,0.,0.},
    {0.,0.,20.,0.},
    {0.,0.,0.,1.}};
  arma::mat R = 0.3*arma::eye(1,1);
  arma::vec umax = {20};
  errorcost<CartPend> cost{Q,R,xd,&syst1};
  sac<CartPend,errorcost<CartPend>> sacsys1 (&syst1,&cost,0.,1.0,umax,unom); 
  mda filt(PI/2, false);
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  ros::init(argc, argv, "impede_test");
  ros::NodeHandle n;
  ImpedeSimulator<CartPend,errorcost<CartPend>,sac<CartPend,errorcost<CartPend>>> sim(&n,&syst1,&cost,&sacsys1,&filt);
  ros::Timer timer = n.createTimer(ros::Duration(DT), &ImpedeSimulator<CartPend,errorcost<CartPend>,sac<CartPend,errorcost<CartPend>>>::timercall, &sim);
  ros::spin();
 return 0;
};