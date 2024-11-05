#include "ros/ros.h"
#include "ros/console.h"
#include "rpg_mpc/mpc_controller.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include "quadrotor_common/geometry_eigen_conversions.h"
#include <boost/make_shared.hpp>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandLongRequest.h>
#include <mavros_msgs/CommandLongResponse.h>
#include <mavlink.h>

std::vector<Eigen::Vector3d> waypoint;
double start_time;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void stateCallback(const nav_msgs::Odometry::ConstPtr& msg, quadrotor_common::QuadStateEstimate& state_estimate) {
  state_estimate.timestamp = msg->header.stamp;
  state_estimate.coordinate_frame = quadrotor_common::QuadStateEstimate::CoordinateFrame::WORLD;

  state_estimate.position = Eigen::Vector3d(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            msg->pose.pose.position.z);
                                          
  state_estimate.orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                  msg->pose.pose.orientation.x,
                                                  msg->pose.pose.orientation.y,
                                                  msg->pose.pose.orientation.z);

  state_estimate.velocity = Eigen::Vector3d(msg->twist.twist.linear.x,
                                            msg->twist.twist.linear.y,
                                            msg->twist.twist.linear.z);

  state_estimate.bodyrates = Eigen::Vector3d(msg->twist.twist.angular.x,
                                             msg->twist.twist.angular.y,
                                             msg->twist.twist.angular.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rpg_mpc");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  rpg_mpc::MpcController<float> controller(nh, pnh);

  quadrotor_common::QuadStateEstimate state_estimate;
  quadrotor_common::Trajectory reference_trajectory;
  rpg_mpc::MpcParams<float> mpc_params;

  ros::Subscriber ros_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, [&state_estimate](const nav_msgs::Odometry::ConstPtr& msg)
                                                                {stateCallback(msg, state_estimate);}
                                                              );

  ros::Publisher input_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
  ros::Publisher reference_pub = nh.advertise<nav_msgs::Path>("reference_trajectory", 1);

  double rate = 50.0;
  ros::Rate loop_rate(rate);
  
  while(ros::ok() && !current_state.connected){
    std::cout << "connecting..." << std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  // initial position
  geometry_msgs::PoseStamped position;
  position.pose.position.x = 0;
  position.pose.position.y = 0;
  position.pose.position.z = 15;
  
  // send a few setpoints before starting to change the mode as OFFBOARD
  for (int i = 200; ros::ok() && i > 0; --i) {
      local_pos_pub.publish(position);
      ros::spinOnce();
      loop_rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  double s_T = ros::Time::now().toSec();
  double c_T = ros::Time::now().toSec();
  geometry_msgs::TwistStamped vel_com;
  vel_com.twist.linear.x = 2.0;
  vel_com.twist.linear.y = 0.0;
  vel_com.twist.linear.z = 0.0;
    
  while (c_T - s_T <= 5.0) {
    local_vel_pub.publish(vel_com);
    ros::spinOnce();
    loop_rate.sleep();
    c_T = ros::Time::now().toSec();
    std::cout << "Elapsed time : " << c_T - s_T << " sec" << '\n';
  }
  
  return 0;
}
