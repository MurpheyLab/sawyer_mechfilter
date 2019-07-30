/*
Kathleen Fitzsimons

This node runs a timer that checks if the current endpoint positon of the Sawyer
is in the home position. It also sends a single command to Sawyer's Motion Control
action server to move the robot to the home state using a joint angle command.

SUBSCRIBERS:
    - cursor_state (/robot/limb/right/endpoint_state)
    - traj_result (/motion/motion_command/result)

PUBLISHERS:N/A
 
SERVICES:
    - Sawyer's MotionControl Action Server is used by setting up
      and action client called (ac) publishing commands to the topic
      /motion/motion_command
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


const double DT=0.5;
const double Nwpt = 1.;

typedef actionlib::SimpleActionClient<intera_motion_msgs::MotionCommandAction> Client;
using namespace std;

class Homing{
  ros::Time t0 = ros::Time::now();
  ros::NodeHandle* nh;
  ros::Subscriber cursor_state;
  ros::Subscriber traj_result;
  bool initcon=false;
    
  string joint_names[7] = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
  intera_motion_msgs::MotionCommandGoal motiongoal;
  //intera_motion_msgs::MotionCommandGoal motionstop; 
  Client* ac = new Client("/motion/motion_command",true);
  geometry_msgs::Pose pose_init;
  geometry_msgs::Pose pose_now;
  
  public:
  Homing(ros::NodeHandle* _nh){
    nh=_nh;
    ROS_INFO("Creating Move to Neutral class");
    //setup publishers & subscribers
    cursor_state = nh->subscribe("/robot/limb/right/endpoint_state",5,&Homing::update_state,this);
    traj_result = nh->subscribe("/motion/motion_command/result",5,&Homing::check_traj,this);
      
    //setup ros action server client
    ac->waitForServer();
    ROS_INFO("Action server started.");
   };
  
  //set up timer callback fxn
  void timercall(const ros::TimerEvent& event){
    if(initcon == true){ROS_INFO("Arm in Home Position");};
    
   };
  
  //state update from end effector
  void update_state(const intera_core_msgs::EndpointState& state){
      pose_now = state.pose;
      double eta = 0.02;
      if(abs(pose_now.position.x-0.6)<eta and abs(pose_now.position.y-0.1)<eta 
        and abs(pose_now.position.z-0.2)<eta
        and abs(pose_now.orientation.x-0.707)<eta and abs(pose_now.orientation.y-0.707)<eta
        and abs(pose_now.orientation.z)<0.01 and abs(pose_now.orientation.w)<0.01){
        pose_init = state.pose;
        if(initcon == false){ROS_INFO("Initcon set");};
        initcon=true;
      };      
           
  };
  
  void check_traj(const intera_motion_msgs::MotionCommandActionResult& res){
    if(!res.result.result){
        ROS_INFO("Tracjectory Failed");
        ROS_INFO("%s",res.result.errorId.c_str());
    }
    else{ROS_INFO("Trajectory Successful");};
    
  };
  
  void set_neutral(){
    motiongoal.command = "start";
    intera_motion_msgs::Trajectory traj;
    traj.joint_names = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
    intera_motion_msgs::Waypoint wpt;
    wpt.active_endpoint = "right_hand";
    wpt.pose.header.stamp = ros::Time::now();
    wpt.pose.header.seq=1;
    wpt.pose.header.frame_id = "base";
    wpt.options.max_joint_speed_ratio = 1.0;
    wpt.options.joint_tolerances = {0.1,0.1,0.1,0.1,0.1,0.1,0.1};
    wpt.options.max_linear_speed = 2.0;
    wpt.options.max_linear_accel = 2.0;
    wpt.options.max_rotational_speed=1.57;
    wpt.options.max_rotational_accel = 1.57;
    wpt.options.corner_distance = 0.5;
    wpt.joint_positions = {-0.106, -0.922, 0.005, 1.771, -0.005, 0.716, 3.218};
    traj.waypoints.push_back(wpt);
    traj.trajectory_options.interpolation_type = "JOINT";
    traj.trajectory_options.interaction_control=false;
    traj.trajectory_options.nso_start_offset_allowed = true;//set to false for 'small' motions
    traj.trajectory_options.end_time = t0 + ros::Duration(5.0);
    traj.trajectory_options.path_interpolation_step = 1./100.;
    motiongoal.trajectory = traj;
    ac->sendGoal(motiongoal);
    ROS_INFO("Moving to Start Position");
  };
        
};
    
int main(int argc, char **argv){
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  ros::init(argc, argv, "gohome");
  ros::NodeHandle n;
  Homing sim(&n);
  sim.set_neutral();
  ros::Timer timer = n.createTimer(ros::Duration(DT), &Homing::timercall, &sim);
  ros::spin();
return 0;
};