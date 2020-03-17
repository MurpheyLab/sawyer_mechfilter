/*
Kathleen Fitzsimons

This node runs a timer that uses the most recent update to the state of Sawyer's 
end effector to accept or reject user inputs. It subscribes to the endpoint_state
and accelerometer of the robot limb and publishes an interaction control command.
A custom message for synchronized data collection is also published, and is used by another
node for visualization of the system in Rviz. 

SUBSCRIBERS:
    - cursor_state (/robot/limb/right/endpoint_state)
    - end_acc (/robot/accelerometer/right_accelerometer/state)

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
#include <std_msgs/Bool.h>
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
#include"SAC_MDA/dklimg_cost.hpp"
#include"SAC_MDA/SAC.hpp"
#include"SAC_MDA/rk4_int.hpp"
#include"SAC_MDA/MIGMDA.hpp"
#include "sawyer_humcpp/mdasys.h"

using namespace std;

const double DT=1./60.;
const double SCALE = 1.0;
const double Kv = 100.0;

template <class system, class objective,class sac,class mda>
class DKLSimulator{
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
  ros::Subscriber reset_flag;
  double xprev[3];
  double yprev[3];
  double veld = 0.0;
  double xvd = 0.,yvd = 0, xvact=0.,yvact=0.;
  sawyer_humcpp::mdasys currstate;
  tf2::Quaternion q_acc, q_ep,q_ep_force;
  int initcon=0;
  
  
  public:
  
  DKLSimulator(ros::NodeHandle* _nh,system *_sys, objective *_cost,sac *_sacsys,mda *_demon){
      nh=_nh;sys = _sys; cost=_cost;sacsys=_sacsys;demon=_demon;
    ROS_INFO("Creating DKLSimulator class");
    //setup publishers & subscribers
    mda_pub = nh->advertise<sawyer_humcpp::mdasys>("mda_topic", 5);  
    interactCommand = nh->advertise<intera_core_msgs::InteractionControlCommand>("/robot/limb/right/interaction_control_command",1);
    cursor_state = nh->subscribe("/robot/limb/right/endpoint_state",5,&DKLSimulator::update_state,this);
    end_acc = nh->subscribe("/robot/accelerometer/right_accelerometer/state",5,&DKLSimulator::calc_input,this);
    reset_flag = nh->subscribe("trial_topic",5,&DKLSimulator::reset_sys,this);
    };
  
  //set up timer callback fxn
  void timercall(const ros::TimerEvent& event){
    tcurr = ros::Time::now() - t0;
    if(initcon==false) {return;};
    if(sys->tcurr>30.){
     interact_options(false);
     interactCommand.publish(interactopt);
     return;};
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
    currstate.sys_time = sys->tcurr;//(ros::Time::now() - t0).toSec();
    double xcurr,ycurr;
    xcurr = SCALE*state.pose.position.x-0.6; ycurr = SCALE*state.pose.position.y-0.1;//center at home position
    //xcurr = (xcurr+0.3)/0.6;//pushing x between 0 and 1 (only needed for fourier version of ergodic metric)
    //ycurr = (ycurr+0.3)/0.6;
    q_ep.setValue(state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w);
    tf2::Quaternion q_force_temp; q_force_temp.setValue(state.wrench.force.x,state.wrench.force.y,state.wrench.force.z,0.0);
    q_ep_force = q_ep*q_force_temp*q_ep.inverse();
    currstate.ef = {state.pose.position.x,state.pose.position.y,state.pose.position.z};
    xprev[0] = xprev[1]; xprev[1]=xprev[2]; xprev[2]=xcurr;//SCALE*state.pose.position.x;
    yprev[0] = yprev[1]; yprev[1]=yprev[2]; yprev[2]=ycurr;//SCALE*state.pose.position.y;
    xvact=(xprev[2]-xprev[1])/DT;
    yvact=(yprev[2]-yprev[1])/DT;
    if(initcon<3){initcon+=1;
      sys->Xcurr = {xprev[2],xvact,yprev[2],yvact}; sys->Ucurr={0.0,0.0}; 
      ROS_INFO("Initcon set");
      }
    else{ 
       
      double xacc = -(2.0*xprev[1]-xprev[2]-xprev[0])/(DT*DT);
      double yacc = -(2.0*yprev[1]-yprev[2]-yprev[0])/(DT*DT);
      currstate.q = {sys->Xcurr[0],sys->Xcurr[2]};
      sys->Ucurr = {xacc,yacc}; //{SCALE*q_acc.x(),SCALE*q_acc.y()};
      arma::mat xdot = sys->f(sys->Xcurr,sys->Ucurr);
      currstate.dq = {(float)xdot(0),(float)xdot(2)};
      currstate.ddq = {(float)xdot(1),(float)xdot(3)};
      currstate.u = {SCALE*q_acc.x(), SCALE*q_acc.y(),xacc,yacc};
      //sacsys->SAC_calc();
      //currstate.sac = {(float)sacsys->ulist(0)};
      currstate.accept = demon->filter(sys->Ucurr);
      //currstate.accept = demon->filter({q_acc.x(),q_acc.y()});
      mda_pub.publish(currstate);
      cost->xmemory(sys->Xcurr);
      sys->step(); 
    };
      
};
  
  void calc_input(const sensor_msgs::Imu& imu){
    tf2::Quaternion q_a_temp;
    q_a_temp.setValue(imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z,0.0);
    q_acc = q_ep*q_a_temp*q_ep.inverse();
  };
  void reset_sys(const std_msgs::Bool& flag){
    if(flag.data==true){
    ROS_INFO("Starting New Trial");
    cost->resample();
    sys->reset();
    cost->t_now=0;
    initcon=0;
    };
    
  };
  //setting the impedance of the interaction options message
  void interact_options(bool reject){
    interactopt.header.stamp = ros::Time::now();
    interactopt.header.seq=1;
    interactopt.header.frame_id = "base";
    interactopt.interaction_control_active = true;
    interactopt.interaction_control_mode = {3,3,1,1,1,1};
    interactopt.K_impedance = {0,0,500,5000,1000,1000};
    interactopt.max_impedance = {false,false,false,false,false,false};
    interactopt.D_impedance = {0,-5,8.,0,2,2};
    interactopt.K_nullspace = {0.,10.,10.,0.,100.,0.,0.};
    interactopt.force_command = {300.,300.,0.,0.,0.,0.};
    if(reject==true){
        interactopt.D_impedance = {(-1^signbit(yvact))*Kv*(yvact-yvd),(-1^signbit(xvact))*Kv*(xvact-xvd),8.,0,2,2};
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
    


arma::vec unom(double t){
  return arma::zeros(2,1);};

int main(int argc, char **argv){
  cv::Mat image;
  double xbound = 0.5,ybound = 0.5;
  string imageName("/home/kt-fitz/sawyer_ws/src/sawyer_humcpp/src/umbrella.png");//need full directory or make sure image is on system PATH
  cv::Mat imagetemp = cv::imread(imageName.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  image = (cv::Scalar::all(255)-imagetemp);
  //cv::flip(image,image,0);
  cv::blur(image,image,cv::Size(100,100));  
  
  DoubleInt syst1 (DT);
  arma::mat R = 0.0001*arma::eye(2,2); double q=1.;
  arma::vec umax = {40.,40.};  
  double T = 0.6;
  arma::mat SIGMA = 0.01*arma::eye(2,2);
  syst1.Xcurr = {0.,0.,0.,0.};//must be initialized before instantiating cost
  dklcost<DoubleInt> cost (q,R,65,SIGMA,0,2,image,xbound,ybound,T,4.0,&syst1);
  sac<DoubleInt,dklcost<DoubleInt>> sacsys1 (&syst1,&cost,0.,T,umax,unom);
  migmda<DoubleInt,dklcost<DoubleInt>> filt(&sacsys1, false);
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  ros::init(argc, argv, "draw_test");
  ros::NodeHandle n;
  DKLSimulator<DoubleInt,dklcost<DoubleInt>,sac<DoubleInt,dklcost<DoubleInt>>,migmda<DoubleInt,dklcost<DoubleInt>>> sim(&n,&syst1,&cost,&sacsys1,&filt);
  ros::Timer timer = n.createTimer(ros::Duration(DT), &DKLSimulator<DoubleInt,dklcost<DoubleInt>,sac<DoubleInt,dklcost<DoubleInt>>,migmda<DoubleInt,dklcost<DoubleInt>>>::timercall, &sim);
  ros::spin();
 return 0;
};