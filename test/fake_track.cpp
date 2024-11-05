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

void send_force_disarm(ros::NodeHandle &nh) {

  ros::ServiceClient command_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // Create the service message
  mavros_msgs::CommandLong srv;
  
  // Set the disarm command (MAV_CMD_COMPONENT_ARM_DISARM)
  srv.request.command = MAV_CMD_COMPONENT_ARM_DISARM;
  srv.request.param1 = 0;       // Param1: 0 = disarm
  srv.request.param2 = 21196;   // Param2: Magic number to force disarm (force flag)
  
  // Call the service
  if (command_client.call(srv)) {
      if (srv.response.success) {
          ROS_INFO("Force disarm command succeeded.");
      } else {
          ROS_WARN("Force disarm command failed.");
      }
  } else {
      ROS_ERROR("Failed to call force disarm service.");
  }
}

quadrotor_common::TrajectoryPoint getFakePoint(double target_vel) {
  quadrotor_common::TrajectoryPoint target_point;
  
  target_point.position.y() = 0.0;
  target_point.position.z() = 2.0;

  double now = ros::Time::now().toSec();
  double duration = now - start_time;

  target_point.position.x() = 15.0 + target_vel*duration;
  
  target_point.velocity.x() = 2.0;
  target_point.velocity.y() = 0.0;
  target_point.velocity.z() = 0.0;

  return target_point;
}

quadrotor_common::Trajectory getDynamicReferenceTrajectory(quadrotor_common::QuadStateEstimate& state_estimate, quadrotor_common::TrajectoryPoint& end_state) {
  Eigen::Vector3d end = end_state.position;
  
  Eigen::Vector3d start = state_estimate.position;
  start.x() += 3.0;
  if (start.z() < end.z() + 1.0) {
    start.z() = end.z();
  } else {
    start.z() -= 1.0;
  }

  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = start;
  start_state.velocity = state_estimate.velocity;

  double wpt_x = (start.x() + end.x()) / 2;
  double wpt_y = (start.y() + end.y()) / 2;
  double wpt_z = (start.z() + end.z()) / 2;

  Eigen::Vector3d wpt(wpt_x, wpt_y, wpt_z);
  waypoint.clear();
  waypoint.push_back(wpt);
  
  Eigen::VectorXd minimization_weights(3);
  minimization_weights << 8, 3, 3;
  
  Eigen::VectorXd segtime(2);
  double seg = (end.x() - start.x()) / 4.0 / 2;
  segtime << seg, seg;

  polynomial_trajectories::PolynomialTrajectorySettings a;
  a.way_points = waypoint;
  a.minimization_weights = minimization_weights;
  a.polynomial_order = 7;
  a.continuity_order = 4;
  
  quadrotor_common::Trajectory dynamic_traj = trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
    segtime, start_state, end_state, a, 10);

  waypoint.push_back(end);
  waypoint.insert(waypoint.begin(), start);
  
  return dynamic_traj;
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

  start_time = ros::Time::now().toSec();

  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "map";
  geometry_msgs::PoseStamped pose;

  quadrotor_common::TrajectoryPoint end_point = getFakePoint(3.0);

  double criteria = abs(end_point.position.x() - state_estimate.position.x());
  while (ros::ok() && criteria > 3.0) {
    std::cout << "Dist: " << criteria << " m" << '\n';
    ros::Time time = ros::Time::now();
    path_msg.header.stamp = time;
    end_point = getFakePoint(3.0);    

    reference_trajectory = getDynamicReferenceTrajectory(state_estimate, end_point);
    criteria = abs(end_point.position.x() - state_estimate.position.x());
    end_point.position.z() = 0.0;
    path_msg.poses.clear();

    double dt = 1.0 / rate;
    int idx = 0;
    for (auto it = reference_trajectory.points.begin(); it != reference_trajectory.points.end(); ++it, ++idx) {
      auto wpt = *it;
      pose.header.stamp = time + ros::Duration(idx * dt);
      pose.header.seq = idx;
      pose.pose.position.x = wpt.position.x();
      pose.pose.position.y = wpt.position.y();
      pose.pose.position.z = wpt.position.z();
      pose.pose.orientation.w = 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      path_msg.poses.push_back(pose);
    }
    reference_pub.publish(path_msg);

    double s_T = ros::Time::now().toSec();
    double c_T = ros::Time::now().toSec();
    while (ros::ok() && c_T - s_T < 0.5) {
      quadrotor_common::TrajectoryPoint e_p = getFakePoint(3.0);
      e_p.position.z() = 0.0;

      quadrotor_common::TrajectoryPoint reference_point = reference_trajectory.points.front();

      double vel_gain = 5.0;
      Eigen::Vector3d vector_current_reference = reference_point.position - state_estimate.position;
      auto& vector = vector_current_reference.normalized();
      auto& vel_vector = vel_gain * vector;

      geometry_msgs::TwistStamped vel_com;
      vel_com.twist.linear.x = vel_vector.x();
      vel_com.twist.linear.y = vel_vector.y();
      vel_com.twist.linear.z = vel_vector.z();

      double dist_reference = vector_current_reference.norm();

      if (dist_reference < 2.0) {
        if (!reference_trajectory.points.empty()) {
          reference_trajectory.points.pop_front();
        }
      } else {
        local_vel_pub.publish(vel_com);
      }
      
      ros::spinOnce();
      
      loop_rate.sleep();
      c_T = ros::Time::now().toSec();
    }
  }
  
  return 0;
}
