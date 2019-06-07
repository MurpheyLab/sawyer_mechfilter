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

const double DT=1./4.;
const double Nwpt = 200.;

typedef actionlib::SimpleActionClient<intera_motion_msgs::MotionCommandAction> Client;
using namespace std;

//template <class system, class objective,class sac>
class MotionSim{
 /* system* sys; 
  objective* cost;
  sac* sacsys;
  mda* demon;*/
  ros::Time t0 = ros::Time::now();
  ros::Duration tcurr=ros::Duration(0);
  ros::NodeHandle* nh;
  intera_core_msgs::InteractionControlCommand interactopt;
  ros::Publisher interactCommand;
  //ros::Publisher mda_pub;
  ros::Subscriber cursor_state;
  ros::Subscriber end_acc;
  //sawyer_humcpp::mdasys currstate;
  tf2::Quaternion q_acc, q_ep,q_ep_force;
  bool initcon=false;
  
  
  string joint_names[7] = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
  intera_motion_msgs::MotionCommandGoal motiongoal;
  Client* ac = new Client("/motion/motion_command",true);
  geometry_msgs::Pose pose_init;
  geometry_msgs::Pose pose_now;
  
  public:
  //MotionSim(ros::NodeHandle* _nh,system *_sys, objective *_cost,sac *_sacsys,mda *_demon){
  MotionSim(ros::NodeHandle* _nh){
    nh=_nh;//sys = _sys; cost=_cost;sacsys=_sacsys;demon=_demon;
    ROS_INFO("Creating MotionSimulator class");
    //setup publishers & subscribers
    //mda_pub = nh->advertise<sawyer_humcpp::mdasys>("mda_topic", 5);  
    interactCommand = nh->advertise<intera_core_msgs::InteractionControlCommand>("/robot/limb/right/interaction_control_command",1);
    cursor_state = nh->subscribe("/robot/limb/right/endpoint_state",5,&MotionSim::update_state,this);
    end_acc = nh->subscribe("/robot/accelerometer/right_accelerometer/state",5,&MotionSim::calc_input,this);
      
    //setup ros action server client
    //ac("/motion/motion_command",true);
    //ROS_INFO("Waiting for action server to start.");
    ac->waitForServer();
    ROS_INFO("Action server started.");
    /*ROS_INFO("Moving to Start Position");
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
    wpt.pose.pose.position.x = 0.6; wpt.pose.pose.position.y = 0.1; wpt.pose.pose.position.z = 0.2;
    wpt.pose.pose.orientation.x = 0.707; wpt.pose.pose.orientation.y = 0.707;
    wpt.pose.pose.orientation.z = 0.; wpt.pose.pose.orientation.w = 0.;
    traj.waypoints.push_back(wpt);
    traj.trajectory_options.interpolation_type = "CARTESIAN";
    traj.trajectory_options.interaction_control=false;
    traj.trajectory_options.nso_start_offset_allowed = true;//set to false for 'small' motions
    traj.trajectory_options.end_time = t0 + ros::Duration(0.5);
    traj.trajectory_options.path_interpolation_step = 1./100.;
    motiongoal.trajectory = traj;
    ac->sendGoal(motiongoal);
    ROS_INFO("In Start Position");
    */
    };
  
  //set up timer callback fxn
  void timercall(const ros::TimerEvent& event){
    tcurr = ros::Time::now() - t0;
    if(initcon==false) {return;};
    motiongoal.command = "start";
    intera_motion_msgs::Trajectory traj;
    traj.joint_names = {"right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
    intera_motion_msgs::Waypoint wpt_next;
    wpt_next.active_endpoint = "right_hand";
    wpt_next.pose.header.stamp = ros::Time::now();
    wpt_next.pose.header.seq=1;
    wpt_next.pose.header.frame_id = "base";
    wpt_next.options.max_joint_speed_ratio = 1.0;
    wpt_next.options.joint_tolerances = {0.5,0.5,0.5,0.5,0.5,0.5,0.5};
    wpt_next.options.max_linear_speed = 5.0;
    wpt_next.options.max_linear_accel = 5.0;
    wpt_next.options.max_rotational_speed=3.0;
    wpt_next.options.max_rotational_accel = 3.0;
    wpt_next.options.corner_distance = 0.5;
    for (int i = 1; i<=Nwpt; i++){
      wpt_next.pose.pose = pose_init;
      wpt_next.pose.pose.position.x = pose_init.position.x - 0.05*sin(tcurr.toSec()+(i*DT/Nwpt));
      wpt_next.pose.pose.position.y = pose_init.position.y + 0.1*(cos(tcurr.toSec()+(i*DT/Nwpt))-1);
      traj.waypoints.push_back(wpt_next);
    }
    traj.trajectory_options.interpolation_type = "CARTESIAN";
    traj.trajectory_options.interaction_control=false;//true; interact_options(true);
    //traj.trajectory_options.interaction_params = interactopt;
    traj.trajectory_options.nso_start_offset_allowed = true;//set to false for 'small' motions
    traj.trajectory_options.tracking_options.goal_time_tolerance = DT;
    traj.trajectory_options.end_time = t0 + tcurr+ ros::Duration(DT);
    traj.trajectory_options.path_interpolation_step = DT;
    motiongoal.trajectory = traj;
    ac->sendGoal(motiongoal);
   };
  
  //state update from end effector
  void update_state(const intera_core_msgs::EndpointState& state){
      pose_now = state.pose;
      if(initcon==false){
        pose_init = state.pose;
        initcon=true;ROS_INFO("Initcon set");
      };      
           
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
  }; 
        
};
    
int main(int argc, char **argv){
  /*Run the main loop, by instatiating a System class, and then
    calling ros::spin*/
  ros::init(argc, argv, "motion_test");
  ros::NodeHandle n;
  MotionSim sim(&n);
  ros::Timer timer = n.createTimer(ros::Duration(DT), &MotionSim::timercall, &sim);
  ros::spin();
  //try putting shutdown commands here?
  //intera_core_msgs::InteractionControlCommand clearimpedance;
  //n.interactCommand.publish()
 return 0;
};