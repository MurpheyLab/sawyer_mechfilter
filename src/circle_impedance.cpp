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
//For specifying a motion command
#include <intera_motion_msgs/MotionCommandAction.h>
#include <actionlib/client/simple_action_client.h>

//Custom includes
#include "sawyer_humcpp/mdasys.h"

const double SCALE = 4.0;
const double DT=0.5;
const double DURATION = 30.;
const double Nwpt = 10.;

typedef actionlib::SimpleActionClient<intera_motion_msgs::MotionCommandAction> Client;
using namespace std;

class CircleMove{
  ros::Time t0; 
  ros::NodeHandle* nh;
  ros::Subscriber cursor_state;
  ros::Subscriber traj_result;
  sawyer_humcpp::mdasys currstate;
  tf2::Quaternion q_acc, q_ep,q_ep_force;
  //bool initcon=false;
    
  string joint_names[7] = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
  intera_motion_msgs::MotionCommandGoal motiongoal;
  //intera_motion_msgs::MotionCommandGoal motionstop; 
  Client* ac = new Client("/motion/motion_command",true);
  geometry_msgs::Pose pose_init;
  geometry_msgs::Pose pose_now;
  
  public:
  CircleMove(ros::NodeHandle* _nh){
    nh=_nh;
    ROS_INFO("Creating CircleMotion class");
    //setup publishers & subscribers
    cursor_state = nh->subscribe("/robot/limb/right/endpoint_state",5,&CircleMove::update_state,this);
    traj_result = nh->subscribe("/motion/motion_command/result",5,&CircleMove::check_traj,this);
      
    //setup ros action server client
    ac->waitForServer();
    ROS_INFO("Action server started.");
   };
  
  //set up timer callback fxn
  void timercall(const ros::TimerEvent& event){
        
   };
  
  //state update from end effector
  void update_state(const intera_core_msgs::EndpointState& state){
    currstate.sys_time = (ros::Time::now() - t0).toSec();
    q_ep.setValue(state.pose.orientation.x,state.pose.orientation.y,state.pose.orientation.z,state.pose.orientation.w);
    tf2::Quaternion q_force_temp; q_force_temp.setValue(state.wrench.force.x,state.wrench.force.y,state.wrench.force.z,0.0);
    q_ep_force = q_ep*q_force_temp*q_ep.inverse();
    currstate.u = {q_ep_force.y(),SCALE*q_acc.y()};
    currstate.ef = {SCALE*state.pose.position.x,SCALE*state.pose.position.y,SCALE*state.pose.position.z};
    };
  
  void check_traj(const intera_motion_msgs::MotionCommandActionResult& res){
    if(!res.result.result){
        ROS_INFO("Tracjectory Failed");
        ROS_INFO("%s\n",res.result.errorId.c_str());
        ROS_INFO("%s\n",res.status.text.c_str());
    }
    else{ROS_INFO("Trajectory Successful");};
    
  };
  
  //setting the impedance of the interaction options message
  /*void interact_options(bool reject){
    interactopt.header.stamp = ros::Time::now();
    interactopt.header.seq=1;
    interactopt.header.frame_id = "base";
    interactopt.interaction_control_active = true;
    interactopt.interaction_control_mode = {1,3,1,1,1,1};
    interactopt.K_impedance = {20,20,1300,1000,1000,1000};
    interactopt.max_impedance = {true,false,true,true,true,true};
    interactopt.D_impedance = {0,0,8.,0,2,2};
    interactopt.K_nullspace = {0.,10.,10.,0.,0.,0.,0.};
    interactopt.force_command = {300.,300.,0.,0.,0.,0.};
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
  };*/
  
  void set_traj(){
    motiongoal.command = "start";
    intera_motion_msgs::Trajectory traj;
    traj.joint_names = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
    intera_motion_msgs::Waypoint wpt_next;
    wpt_next.active_endpoint = "right_hand";
    t0 = ros::Time::now();
    wpt_next.pose.header.stamp = t0;
    wpt_next.pose.header.seq=1;
    wpt_next.pose.header.frame_id = "base";
    wpt_next.options.max_joint_speed_ratio = 1.0;
    wpt_next.options.joint_tolerances = {0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    wpt_next.options.max_linear_speed = 2.0;
    wpt_next.options.max_linear_accel = 2.0;
    wpt_next.options.max_rotational_speed=1.57;
    wpt_next.options.max_rotational_accel = 1.57;
    wpt_next.options.corner_distance = 0.5;
    wpt_next.pose.pose.position.z = 0.2; 
    wpt_next.pose.pose.orientation.x=0.707; wpt_next.pose.pose.orientation.y=0.707; 
    wpt_next.pose.pose.orientation.z=0.0; wpt_next.pose.pose.orientation.w=0.0;
    for (int i = 1; i<=Nwpt; i++){
      wpt_next.pose.pose.position.x = 0.6 - 0.25*sin((i*DURATION/Nwpt)/4);
      wpt_next.pose.pose.position.y = 0.1 + 0.3*(cos((i*DURATION/Nwpt)/4)-1);
      traj.waypoints.push_back(wpt_next);
    }
    traj.trajectory_options.interpolation_type = "CARTESIAN";
    traj.trajectory_options.interaction_control=false;
    traj.trajectory_options.nso_start_offset_allowed = true;//set to false for 'small' motions
    traj.trajectory_options.end_time = t0 + ros::Duration(DURATION);
    motiongoal.trajectory = traj;
    ac->sendGoal(motiongoal);
    ROS_INFO("Starting Trajectory");
    
         
  };
        
};
    
int main(int argc, char **argv){
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  ros::init(argc, argv, "circle");
  ros::NodeHandle n;
  CircleMove sim(&n);
  sim.set_traj();
  ros::Timer timer = n.createTimer(ros::Duration(DT), &CircleMove::timercall, &sim);
  ros::spin();
return 0;
};