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
#include <opencv2/opencv.hpp>
#include"SAC_MDA/doubleint.hpp"
#include"SAC_MDA/ergodic_cost.hpp"
#include"SAC_MDA/SAC.hpp"
#include"SAC_MDA/rk4_int.hpp"
#include"SAC_MDA/MIGMDA.hpp"
#include "sawyer_humcpp/mdasys.h"

using namespace std;

const double DT=1./100.;
const double SCALE = 1.0;
const double Kv = 150.0;

template <class system, class objective,class sac,class mda>
class DrawSimulator{
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
  float yprev[3];
  float veld = 0.0;
  float xvd = 0.,yvd = 0, xvact=0.,yvact=0.;
  sawyer_humcpp::mdasys currstate;
  tf2::Quaternion q_acc, q_ep,q_ep_force;
  bool initcon=false;
  
  
  public:
  
  DrawSimulator(ros::NodeHandle* _nh,system *_sys, objective *_cost,sac *_sacsys,mda *_demon){
      nh=_nh;sys = _sys; cost=_cost;sacsys=_sacsys;demon=_demon;
    ROS_INFO("Creating DrawSimulator class");
    //setup publishers & subscribers
    mda_pub = nh->advertise<sawyer_humcpp::mdasys>("mda_topic", 5);  
    interactCommand = nh->advertise<intera_core_msgs::InteractionControlCommand>("/robot/limb/right/interaction_control_command",1);
    cursor_state = nh->subscribe("/robot/limb/right/endpoint_state",5,&DrawSimulator::update_state,this);
    end_acc = nh->subscribe("/robot/accelerometer/right_accelerometer/state",5,&DrawSimulator::calc_input,this);
    
    };
  
  //set up timer callback fxn
  void timercall(const ros::TimerEvent& event){
    tcurr = ros::Time::now() - t0;
    if(initcon==false) {return;};
    if(currstate.accept){
      interact_options(false);
      xvd = (xprev[2]-xprev[1])/DT;
      yvd = (yprev[2]-yprev[1])/DT;
    }
      else{interact_options(true);};
    interactCommand.publish(interactopt);
   };
  
  //state update from end effector
  void update_state(const intera_core_msgs::EndpointState& state){
      currstate.sys_time = (ros::Time::now() - t0).toSec();
      q_ep.setValue(state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w);
      tf2::Quaternion q_force_temp; q_force_temp.setValue(state.wrench.force.x,state.wrench.force.y,state.wrench.force.z,0.0);
      q_ep_force = q_ep*q_force_temp*q_ep.inverse();
      currstate.u = {SCALE*q_acc.y(), SCALE*q_acc.x()};
      currstate.ef = {SCALE*(float)state.pose.position.x,SCALE*(float)state.pose.position.y,SCALE*(float)state.pose.position.z};
      if(initcon==false)
        {sys->Xcurr = {SCALE*state.pose.position.x,0.,SCALE*state.pose.position.y,0.}; sys->Ucurr={0.0,0.0}; 
         initcon=true;ROS_INFO("Initcon set");
         xprev[0]=SCALE*state.pose.position.y; xprev[1]=SCALE*state.pose.position.y; xprev[2]=SCALE*state.pose.position.y;
         yprev[0]=SCALE*state.pose.position.x;yprev[1]=SCALE*state.pose.position.x; yprev[2]=SCALE*state.pose.position.x;
        };      
      xprev[0] = xprev[1]; xprev[1]=xprev[2]; xprev[2]=SCALE*(float)state.pose.position.y;
      yprev[0] = yprev[1]; yprev[1]=yprev[2]; yprev[2]=SCALE*(float)state.pose.position.x;
      xvact=(xprev[2]-xprev[1])/DT;
      yvact=(yprev[2]-yprev[1])/DT;
      float xacc = -(2.0*xprev[1]-xprev[2]-xprev[0])/(DT*DT);
      float yacc = -(2.0*yprev[1]-yprev[2]-yprev[0])/(DT*DT);
      currstate.q = {(float)sys->Xcurr[0],(float)sys->Xcurr[2]};
      arma::mat xdot = sys->f(sys->Xcurr,sys->Ucurr);
      currstate.dq = {(float)xdot(0),(float)xdot(2)};
      currstate.ddq = {(float)xdot(1),(float)xdot(3)};
      sys->Ucurr = {xacc,yacc}; 
      sys->step();
      //sacsys->SAC_calc();
      //currstate.sac = {(float)sacsys->ulist(0)};
      currstate.accept = demon->filter(sys->Ucurr);
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
    interactopt.max_impedance = {false,false,true,true,true,true};
    interactopt.D_impedance = {0,0,8.,0,2,2};
    interactopt.K_nullspace = {0.,10.,10.,0.,0.,0.,0.};
    interactopt.force_command = {0.,0.,0.,0.,0.,0.};
    if(reject==true){
        interactopt.D_impedance = {50*(yvact-yvd),50*(xvact-xvd),8.,0,2,2};
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
    
cv::Mat image;
//double imgTotal=0.;
double xbound = 0.5,ybound = 0.5;//2200/2;
double phid(double x1, double x2){
  double ind1 = x2*2200.; double ind2 = x1*2200.;
  double intensity = image.at<uchar>(round(ind1),round(ind2));
  double totalInt = cv::mean(image)[0]*(xbound*2)*(ybound*2);//cout<<totalInt<<" ";
  intensity = intensity/totalInt;//(255*7);
  return intensity;};

arma::vec unom(double t){
  return arma::zeros(2,1);};

int main(int argc, char **argv){
  string imageName("apple.png");
  cv::Mat imagetemp = cv::imread(imageName.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  image = (cv::Scalar::all(255)-imagetemp);cout<<cv::mean(image)[0]<<"\n";
  cv::flip(image,image,-1);  
  DoubleInt syst1 (1./60.);
  arma::mat R = 0.01*arma::eye(2,2); double q=1000.;
  arma::vec umax = {40,40};  
  double T = 1.0;
  ergodicost<DoubleInt> cost (q,R,10,0,2,phid,xbound,ybound,T,&syst1);ROS_INFO("here");
  sac<DoubleInt,ergodicost<DoubleInt>> sacsys1 (&syst1,&cost,0.,T,umax,unom);
  migmda<DoubleInt,ergodicost<DoubleInt>> filt(&sacsys1, false);
  ROS_INFO("here");
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  ros::init(argc, argv, "draw_test");
  ros::NodeHandle n;
  DrawSimulator<DoubleInt,ergodicost<DoubleInt>,sac<DoubleInt,ergodicost<DoubleInt>>,migmda<DoubleInt,ergodicost<DoubleInt>>> sim(&n,&syst1,&cost,&sacsys1,&filt);
  ros::Timer timer = n.createTimer(ros::Duration(DT), &DrawSimulator<DoubleInt,ergodicost<DoubleInt>,sac<DoubleInt,ergodicost<DoubleInt>>,migmda<DoubleInt,ergodicost<DoubleInt>>>::timercall, &sim);
  ros::spin();
 return 0;
};