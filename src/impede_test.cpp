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
#include <intera_core_msgs/InteractionControlCommand.h>
#include <intera_core_msgs/EndpointState.h>
#include <sensor_msgs/Imu.h>
//#include <intera_interface/RobotEnable.h>
//#include<intera_interface>


//from intera_motion_interface import InteractionOptions
//from intera_motion_interface.utility_functions import int2bool

//#include "intera_interface"
//from intera_interface import CHECK_VERSION


class ImpedeSimulator{
  ros::Time t0 = ros::Time::now();
  ros::Duration tcurr=ros::Duration(0);
  ros::NodeHandle* nh;
  intera_core_msgs::InteractionControlCommand interactopt;
  //setup publishers, subscribers:
  ros::Publisher interactCommand;
  ros::Subscriber cursor_state;
  ros::Subscriber end_acc;
  int counter = 1;
  //void interact_options(bool);
    
  public:
  ImpedeSimulator(ros::NodeHandle* _nh){
      nh=_nh;
    ROS_INFO("Creating ImpedeSimulator class");
    //setup publishers
    interactCommand = nh->advertise<intera_core_msgs::InteractionControlCommand>("/robot/limb/right/interaction_control_command",1);
    cursor_state = nh->subscribe("/robot/limb/right/endpoint_state",5,&ImpedeSimulator::update_state,this);
    end_acc = nh->subscribe("/robot/accelerometer/right_accelerometer/state",5,&ImpedeSimulator::calc_input,this);
    };
  
  //set up timer callback fxn
  void timercall(const ros::TimerEvent& event){
    tcurr = ros::Time::now() - t0;
    if(tcurr.toSec()>26.){interact_options(false);ROS_INFO("UnLocked");}
    else if(tcurr.toSec()>25.){interact_options(true);ROS_INFO("Locked");}
    else if(tcurr.toSec()>17.){interact_options(false);ROS_INFO("UnLocked");}
    else if(tcurr.toSec()>16.){interact_options(true);ROS_INFO("Locked");}
    else if(tcurr.toSec()>12.){interact_options(false);ROS_INFO("UnLocked");}
    else if(tcurr.toSec()>10.){interact_options(true); ROS_INFO("Locked");} 
    else{interact_options(false);};
    interactCommand.publish(interactopt); 
    
    //ROS_INFO("Time Now: %f",tcurr.toSec());
  };
  //state update from end effector
  void update_state(const intera_core_msgs::EndpointState& state){
    //ROS_INFO("Got the State");
  };
  //state update from end effector
  void calc_input(const sensor_msgs::Imu& imu){
    //ROS_INFO("Got the Acc");
  };
  //setting the impedance of the interaction options message
  void interact_options(bool reject){
    interactopt.header.stamp = ros::Time::now();
    interactopt.header.seq=1;
    interactopt.header.frame_id = "base";
    interactopt.interaction_control_active = true;
    interactopt.interaction_control_mode = {1,1,1,1,1,1};
    //if (reject==true){  counter++;
    //interactopt.K_impedance ={counter*10.,counter*10.,1300,30,30,30} ;
    //}
    //else 
    interactopt.K_impedance = {0,0,1300,30,30,30};
    interactopt.max_impedance = {false,false,true,true,true,true};
    interactopt.D_impedance = {0,0,8.,0,2,2};
    if(reject==true)interactopt.D_impedance = {100.,100.,50.,2.,2.,2.};
    interactopt.K_nullspace = {0.,10.,10.,0.,0.,0.,0.};
    interactopt.force_command = {0.,0.,0.,0.,0.,0.};
    interactopt.interaction_frame.position.x = 0;
    interactopt.interaction_frame.position.y =0;
    interactopt.interaction_frame.position.z  =0;
    interactopt.interaction_frame.orientation.x = 0;
    interactopt.interaction_frame.orientation.y = 0;
    interactopt.interaction_frame.orientation.z = 0;
    interactopt.interaction_frame.orientation.w = 1;
    interactopt.endpoint_name ="right_hand";
    interactopt.in_endpoint_frame = false;
    interactopt.disable_damping_in_force_control = false;
    interactopt.disable_reference_resetting = false;
    //if(reject==true)interactopt.disable_reference_resetting = false;
    interactopt.rotations_for_constrained_zeroG = false;
  }; 
        
};
    
int main(int argc, char **argv){
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  ros::init(argc, argv, "impede_test");
  ros::NodeHandle n;
  ImpedeSimulator sim(&n);
  ros::Timer timer = n.createTimer(ros::Duration(1./100.), &ImpedeSimulator::timercall, &sim);
  ros::spin();
 return 0;
};