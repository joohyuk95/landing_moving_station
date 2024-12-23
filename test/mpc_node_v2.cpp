/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

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

visualization_msgs::MarkerArray visualizer2(std::vector<Eigen::Vector3d>& msg);

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
  target_point.position.z() = 5.0;

  double now = ros::Time::now().toSec();
  double duration = now - start_time;

  target_point.position.x() = 20.0 + target_vel*duration;

  return target_point;
}

quadrotor_common::Trajectory getDynamicReferenceTrajectory(Eigen::Vector3d& start, quadrotor_common::TrajectoryPoint& end_state) {
  Eigen::Vector3d end = end_state.position;
  
  start.x() += 3.0;
  if (start.z() < end.z() + 1.0) {
    start.z() = end.z();
  } else {
    start.z() -= 1.0;
  }

  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = start;

  double wpt_x = (start.x() + end.x()) / 2;
  double wpt_y = (start.y() + end.y()) / 2;
  double wpt_z = (start.z() + end.z()) / 2;

  Eigen::Vector3d wpt(wpt_x, wpt_y, wpt_z);
  waypoint.clear();
  waypoint.push_back(wpt);
  
  Eigen::VectorXd minimization_weights(3);
  minimization_weights << 8, 3, 3;
  
  Eigen::VectorXd segtime(2);
  segtime << 3, 3;

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

quadrotor_common::Trajectory getReferenceTrajectory() {
  // const double max_vel = 7.0;
  // const double max_thrust = 20.0;
  // const double max_roll_pitch_rate = 2.0;

  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = Eigen::Vector3d(0.0, 0.0, 20.0);
  // start_state.orientation = Eigen::Quaterniond(0.246, 0.669, 0.487, 0.504);
  // start_state.heading = 0.0;
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = Eigen::Vector3d(40.0, 0.0, 5.0);
  // end_state.orientation = Eigen::Quaterniond(0.399, 0.365, 0.365, 0.758);
  // end_state.heading = M_PI;

  // quadrotor_common::Trajectory fixed_traj = trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
  //   start_state, end_state, 5, max_vel, max_thrust, max_roll_pitch_rate, 10);

  // quadrotor_common::Trajectory fixed_traj = trajectory_generation_helper::polynomials::computeFixedTimeTrajectory(
  //   start_state, end_state, 5, 10, 10);


  // std::vector<Eigen::Vector3d> waypoint;
  Eigen::Vector3d wpt1(20, 0, 7);
  // Eigen::Vector3d wpt2(30, 0, 5);
  // Eigen::Vector3d wpt3(20, 0, 5);
  // Eigen::Vector3d wpt4(30, 0, 5);
  
  waypoint.push_back(wpt1);
  // waypoint.push_back(wpt2);
  // waypoint.push_back(wpt3);
  // waypoint.push_back(wpt4);
  // Eigen::VectorXd minimization_weights(3);  // n waipoints (including start, end), n-1 weights
  // minimization_weights << 10, 10, 2;
  
  // Eigen::VectorXd segtime(3);
  // segtime << 8,3,3;
  Eigen::VectorXd minimization_weights(2);
  minimization_weights << 10, 10;
  
  Eigen::VectorXd segtime(2);
  segtime << 10, 10;

  polynomial_trajectories::PolynomialTrajectorySettings a;
  a.way_points = waypoint;
  a.minimization_weights = minimization_weights;
  a.polynomial_order = 4;
  a.continuity_order = 4;
  
  quadrotor_common::Trajectory fixed_traj = trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
    segtime, start_state, end_state, a, 10);

  // trajectory_generation_helper::heading::addConstantHeadingRate(
  //   start_state.heading, end_state.heading, &fixed_traj);
  waypoint.push_back(end_state.position);
  waypoint.insert(waypoint.begin(), start_state.position);
  

  return fixed_traj;
}

mavros_msgs::AttitudeTarget command2Bodyrate(const quadrotor_common::ControlCommand& command) {
  quadrotor_msgs::ControlCommand ros_command = command.toRosMessage();
  
  mavros_msgs::AttitudeTarget attitude;
  
  attitude.header.stamp = ros_command.header.stamp;
  attitude.header.frame_id = "map";

  attitude.type_mask = 128;

  // attitude.orientation.x = ros_command.orientation.x;
  // attitude.orientation.y = ros_command.orientation.y;
  // attitude.orientation.z = ros_command.orientation.z;
  // attitude.orientation.w = ros_command.orientation.w;
  attitude.orientation.w = 1.0;
  attitude.body_rate.x = ros_command.bodyrates.x;
  attitude.body_rate.y = ros_command.bodyrates.y;
  attitude.body_rate.z = ros_command.bodyrates.z;

  const auto& thrust = ros_command.collective_thrust;
  attitude.thrust = 0.1 + (thrust - 2.0) * (0.7 - 0.1) / (20.0 - 2.0);
  // attitude.thrust = ros_command.collective_thrust;
  return attitude;
}

visualization_msgs::MarkerArray visualizer(nav_msgs::Path& msg) { // trajectory path components
    // Clear the previous marker array
    visualization_msgs::MarkerArray marker_array_;

    // Create a Marker for each point in the Path (visualized as spheres)
    for (size_t i = 0; i < msg.poses.size(); ++i)
    {
        // Create a Marker for the point (as a sphere)
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = "map";
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "path_points";
        point_marker.id = i;
        point_marker.type = visualization_msgs::Marker::SPHERE;
        point_marker.action = visualization_msgs::Marker::ADD;

        // Set the position of the sphere to the path point
        point_marker.pose.position = msg.poses[i].pose.position;
        point_marker.pose.orientation.w = 1.0;
        // Set the scale of the sphere
        point_marker.scale.x = 0.1;
        point_marker.scale.y = 0.1;
        point_marker.scale.z = 0.1;

        // Set the color of the sphere (red)
        point_marker.color.r = 1.0;
        point_marker.color.g = 0.0;
        point_marker.color.b = 0.0;
        point_marker.color.a = 1.0;

        // Add the point marker to the MarkerArray
        marker_array_.markers.push_back(point_marker);
    }

    // Publish the MarkerArray
    return marker_array_;
}

visualization_msgs::Marker visualizer1(quadrotor_common::TrajectoryPoint& msg) { // target point
    // Create a Marker for the path points (visualized as spheres)
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "map"; // Set the frame ID
    point_marker.header.stamp = ros::Time::now(); // Set the current time
    point_marker.ns = "target_point"; // Namespace
    point_marker.id = 0; // Unique ID for this marker
    point_marker.type = visualization_msgs::Marker::SPHERE; // Use SPHERE_LIST type
    point_marker.action = visualization_msgs::Marker::ADD; // Action to add marker

    point_marker.pose.position.x = msg.position.x();
    point_marker.pose.position.y = msg.position.y();
    point_marker.pose.position.z = msg.position.z();
    point_marker.pose.orientation.w = 1.0;

    // Set the scale of the spheres
    point_marker.scale.x = 0.15;  // Radius of each sphere
    point_marker.scale.y = 0.15;
    point_marker.scale.z = 0.15;

    // Set the color of the spheres (red)
    point_marker.color.r = 0.0;
    point_marker.color.g = 0.0;
    point_marker.color.b = 1.0;
    point_marker.color.a = 1.0; // Fully opaque

    return point_marker; // Return the constructed marker
}

visualization_msgs::MarkerArray visualizer2(std::vector<Eigen::Vector3d>& msg) { // Waypoints
    // Create a Marker for the path points (visualized as spheres)
    visualization_msgs::MarkerArray waypoints;
    for (size_t i = 0; i < msg.size(); ++i)
    {
      visualization_msgs::Marker waypoint;
      waypoint.header.frame_id = "map"; // Set the frame ID
      waypoint.header.stamp = ros::Time::now(); // Set the current time
      waypoint.ns = "waypoint"; // Namespace
      waypoint.id = i; // Unique ID for this marker
      waypoint.type = visualization_msgs::Marker::SPHERE; // Use SPHERE_LIST type
      waypoint.action = visualization_msgs::Marker::ADD; // Action to add marker

      waypoint.pose.position.x = msg[i].x();
      waypoint.pose.position.y = msg[i].y();
      waypoint.pose.position.z = msg[i].z();
      waypoint.pose.orientation.w = 1.0;

      // Set the scale of the spheres
      waypoint.scale.x = 0.3;  // Radius of each sphere
      waypoint.scale.y = 0.3;
      waypoint.scale.z = 0.3;

      // Set the color of the spheres (green)
      waypoint.color.r = 0.0;
      waypoint.color.g = 1.0;
      waypoint.color.b = 0.0;
      waypoint.color.a = 1.0; // Fully opaque
      waypoints.markers.push_back(waypoint);
    }
    return waypoints; // Return the constructed marker
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

  ros::Publisher marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
  ros::Publisher marker_array_pub_1 = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array1", 10);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
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
  position.pose.position.z = 20;
  
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
    ros::Time time = ros::Time::now();
    path_msg.header.stamp = time;
    end_point = getFakePoint(3.0);
    reference_trajectory = getDynamicReferenceTrajectory(state_estimate.position, end_point);
    criteria = abs(end_point.position.x() - state_estimate.position.x());

    visualization_msgs::MarkerArray wpt = visualizer2(waypoint);
    marker_array_pub_1.publish(wpt);

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
    visualization_msgs::MarkerArray marker = visualizer(path_msg);
    marker_array_pub_.publish(marker);

    double s_T = ros::Time::now().toSec();
    double c_T = ros::Time::now().toSec();
    while (ros::ok() && c_T - s_T < 0.5) {

      quadrotor_common::TrajectoryPoint reference_point = reference_trajectory.points.front();
      visualization_msgs::Marker target = visualizer1(reference_point);
      marker_pub.publish(target);

      double vel_gain = 4.0;
      Eigen::Vector3d vector_current_reference = reference_point.position - state_estimate.position;
      auto& vector = vector_current_reference.normalized();
      auto& vel_vector = vel_gain * vector;

      geometry_msgs::TwistStamped vel_com;
      vel_com.twist.linear.x = vel_vector.x();
      vel_com.twist.linear.y = vel_vector.y();
      vel_com.twist.linear.z = vel_vector.z();

      double dist_reference = vector_current_reference.norm();
      // std::cout << "state: " << "x: " <<  state_estimate.position.x() << ", y: " <<  state_estimate.position.y() << ", z: " <<  state_estimate.position.z() << '\n';
      // std::cout << "refer: " << "x: " <<  reference_point.position.x() << ", y: " <<  reference_point.position.y() << ", z: " <<  reference_point.position.z() << '\n';
      // std::cout << "dist: " << dist_reference << '\n';

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
  send_force_disarm(nh);

  return 0;
}
