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
#include "geometry_msgs/Point.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
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
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <gazebo_msgs/ModelStates.h> // SITL
#include <string> // SITL
#include <cstdlib>

// #define VEL_GAIN 5.0
#define HEIGHT_OFFSET 2.0
#define INIT_HEIGHT 5.0
#define SITL 1
#define MARKER_OFFSET 1.0

std::vector<Eigen::Vector3d> waypoint;
// double start_time = 0;
std::mutex mtx;
bool parallel = false;
bool is_detected = false;
double vel_gain = 5.0;
double vel_offset = 2.0;
double vz_max = 0.7;

mavros_msgs::State current_state;
geometry_msgs::Point station_pos;
geometry_msgs::TwistStamped station_vel;
geometry_msgs::PoseStamped aruco_pose;
double hdg; // station heading
double hd;  // drone heading

class PID {
public:
    PID(double kp, double ki, double kd, double vmax)
        : kp_(kp), ki_(ki), kd_(kd), v_max(vmax), 
        prev_error_(Eigen::Vector3d::Zero()), integral_(Eigen::Vector3d::Zero()) {}

    Eigen::Vector3d calculate(const Eigen::Vector3d& setpoint, const Eigen::Vector3d& current, double dt) {
        Eigen::Vector3d error = setpoint - current;
        integral_ += error * dt;
        Eigen::Vector3d derivative = (error - prev_error_) / dt;
        
        Eigen::Vector3d pid_output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        
        double raw_norm = pid_output.norm();
        
        if (raw_norm > v_max) {
          pid_output = v_max*(pid_output.normalized());
          integral_ -= error * dt;
        }
        
        prev_error_ = error;

        return pid_output;
    }

    void setVMax(double vmax) {
        v_max = vmax;
    }

private:
    double kp_, ki_, kd_, v_max;
    Eigen::Vector3d prev_error_;
    Eigen::Vector3d integral_;
};

void aruco_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    aruco_pose = *msg;
    
    double detect = aruco_pose.pose.orientation.z;
    if (detect == -1) {
      is_detected = false;
    } else {
      is_detected = true;
    }
    
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void enu_pos_cb(const geometry_msgs::Point::ConstPtr& msg){
    station_pos = *msg;
}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {   // SITL
    std::string target_model = "husky";

    auto it = std::find(msg->name.begin(), msg->name.end(), target_model);
    if (it != msg->name.end()) {        
        int index = std::distance(msg->name.begin(), it);
        
        station_pos = msg->pose[index].position;
    } else {
        ROS_WARN("Iris model not found in ModelStates topic.");
    }
}

void enu_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    station_vel = *msg;
    vel_gain = std::sqrt(std::pow(station_vel.twist.linear.x, 2) + std::pow(station_vel.twist.linear.y, 2)) + vel_offset;    
}

void hdg_cb(const std_msgs::Float64::ConstPtr& msg) {
    hdg = msg->data;
}

void hd_cb(const std_msgs::Float64::ConstPtr& msg) {
    hd = msg->data;
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

constexpr double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

Eigen::Vector2d getENUUnitVector() {
    // Convert azimuth angle to radians
    double azimuth_radians = degreesToRadians(hdg);

    // Calculate East (x) and North (y) components
    double east = sin(azimuth_radians);  // x component (East)
    double north = cos(azimuth_radians); // y component (North)

    // Return the unit vector in the EN plane
    return Eigen::Vector2d(east, north);
}

double getAzimuth(const double& east, const double& north) {
  double azimuth = atan2(east, north) * (180.0 / M_PI);
  if (azimuth < 0) {
    azimuth += 360;
  }
  return azimuth;
}

Eigen::Vector2d findFootOfPerpendicular(const Eigen::Vector3d& last_target) {

    Eigen::Vector2d linePoint(station_pos.x, station_pos.y);
    Eigen::Vector2d direction = getENUUnitVector();
    Eigen::Vector2d point(last_target.x(), last_target.y());

    // Calculate the vector from linePoint to the point
    Eigen::Vector2d PQ = point - linePoint;
    
    // Project PQ onto the direction vector
    double projectionLength = PQ.dot(direction) / direction.dot(direction);
    Eigen::Vector2d projection = projectionLength * direction;
    
    // The foot of the perpendicular
    Eigen::Vector2d foot = linePoint + projection;
    
    return foot;
}

quadrotor_common::TrajectoryPoint getFakePoint(double target_vel) { // read target position
  quadrotor_common::TrajectoryPoint target_point;
  
  Eigen::Vector2d vec = getENUUnitVector();
  auto& vector = vec.normalized();
  auto& vel = vel_gain * vector;

  target_point.position.x() = station_pos.x - (MARKER_OFFSET*vector.x());
  target_point.position.y() = station_pos.y - (MARKER_OFFSET*vector.y());
  target_point.position.z() = HEIGHT_OFFSET;

  target_point.velocity.x() = vel.x();
  target_point.velocity.y() = vel.y();
  target_point.velocity.z() = 0.0;

  return target_point;
}

// quadrotor_common::TrajectoryPoint getFakePoint(double target_vel) {  // virtual point
//   quadrotor_common::TrajectoryPoint target_point;
  
//   target_point.position.y() = 0.0;
//   target_point.position.z() = 2.0;

//   double now = ros::Time::now().toSec();
//   double duration = now - start_time;

//   target_point.position.x() = 20.0 + target_vel*duration;
  
//   target_point.velocity.x() = target_vel;
//   target_point.velocity.y() = 0.0;
//   target_point.velocity.z() = 0.0;

//   return target_point;
// }

quadrotor_common::Trajectory getDynamicReferenceTrajectory(quadrotor_common::QuadStateEstimate& state_estimate, quadrotor_common::TrajectoryPoint& end_state) { // 현재 위치를 경로 시작점으로
  Eigen::Vector3d end = end_state.position;
  Eigen::Vector3d start = state_estimate.position;
  
  // double tracking_vel = 5.0;
  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = start;
  // start_state.velocity = state_estimate.velocity;
  // start_state.velocity.z() = 0.0;
  Eigen::Vector2d foot_pt = findFootOfPerpendicular(start);

  double wpt_x = (2*foot_pt.x() + end.x()) / 3;
  double wpt_y = (2*foot_pt.y() + end.y()) / 3;
  double wpt_z = (2*start.z() + end.z()) / 3;

  Eigen::Vector3d wpt(wpt_x, wpt_y, wpt_z);

  double tracking_vel = vel_gain;
  start_state.velocity = tracking_vel*((wpt - start).normalized());

  waypoint.clear();
  waypoint.push_back(wpt);
  
  Eigen::VectorXd minimization_weights(3);
  minimization_weights << 8, 3, 3;
  
  Eigen::VectorXd segtime(2);
  double seg = ((end - start).norm() / tracking_vel) / 3;
  segtime << seg, 2*seg;

  polynomial_trajectories::PolynomialTrajectorySettings a;
  a.way_points = waypoint;
  a.minimization_weights = minimization_weights;
  a.polynomial_order = 6;
  a.continuity_order = 3;
  
  quadrotor_common::Trajectory dynamic_traj = trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
    segtime, start_state, end_state, a, 10);

  waypoint.push_back(end);
  waypoint.insert(waypoint.begin(), start);
  
  return dynamic_traj;
}

// 이전 경로의 마지막 target point를 다음 경로 시작점으로
quadrotor_common::Trajectory getDynamicReferenceTrajectory2(quadrotor_common::QuadStateEstimate& state_estimate, quadrotor_common::TrajectoryPoint& start_state, quadrotor_common::TrajectoryPoint& end_state) {
  Eigen::Vector3d start = start_state.position;
  // start_state.velocity = state_estimate.velocity;
  Eigen::Vector3d end = end_state.position;
  // double tracking_vel = 5.0;
  Eigen::Vector2d foot_pt = findFootOfPerpendicular(start);

  double wpt_x = (2*foot_pt.x() + end.x()) / 3;
  double wpt_y = (2*foot_pt.y() + end.y()) / 3;
  double wpt_z = (2*start.z() + end.z()) / 3;

  double tracking_vel = vel_gain;
  Eigen::Vector3d wpt(wpt_x, wpt_y, wpt_z);
  start_state.velocity = tracking_vel*((wpt - start).normalized());

  waypoint.clear();
  waypoint.push_back(wpt);
  
  Eigen::VectorXd minimization_weights(3);
  minimization_weights << 8, 3, 3;
  
  Eigen::VectorXd segtime(2);
  double seg = ((end - start).norm() / tracking_vel) / 3;
  segtime << seg, 2*seg;

  polynomial_trajectories::PolynomialTrajectorySettings a;
  a.way_points = waypoint;
  a.minimization_weights = minimization_weights;
  a.polynomial_order = 6;
  a.continuity_order = 3;
  
  quadrotor_common::Trajectory dynamic_traj = trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
    segtime, start_state, end_state, a, 10);

  waypoint.push_back(end);
  waypoint.insert(waypoint.begin(), start);
  
  return dynamic_traj;
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

visualization_msgs::MarkerArray visualizer(nav_msgs::Path& msg, int add) { // trajectory path components
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
        if (add) {
          point_marker.action = visualization_msgs::Marker::ADD;
        } else {
          point_marker.action = visualization_msgs::Marker::DELETE;
        }

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
    // point_marker.id = 0; // Unique ID for this marker
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

    // Set the color of the spheres (blue)
    point_marker.color.r = 0.0;
    point_marker.color.g = 0.0;
    point_marker.color.b = 1.0;
    point_marker.color.a = 1.0; // Fully opaque

    return point_marker; // Return the constructed marker
}

visualization_msgs::Marker visualizer11(quadrotor_common::TrajectoryPoint& msg) { // Fake target (cube type)
    // Create a Marker for the path points (visualized as spheres)
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "map"; // Set the frame ID
    point_marker.header.stamp = ros::Time::now(); // Set the current time
    point_marker.ns = "moving_object"; // Namespace
    point_marker.id = 0; // Unique ID for this marker
    point_marker.type = visualization_msgs::Marker::CUBE; // Use SPHERE_LIST type
    point_marker.action = visualization_msgs::Marker::ADD; // Action to add marker

    point_marker.pose.position.x = msg.position.x();
    point_marker.pose.position.y = msg.position.y();
    point_marker.pose.position.z = msg.position.z();
    point_marker.pose.orientation.w = 1.0;

    // Set the scale of the spheres
    point_marker.scale.x = 1.0;  // Radius of each sphere
    point_marker.scale.y = 1.0;
    point_marker.scale.z = 0.1;

    // Set the color of the spheres (yellow)
    point_marker.color.r = 1.0;
    point_marker.color.g = 1.0;
    point_marker.color.b = 0.0;
    point_marker.color.a = 1.0; // Fully opaque

    return point_marker; // Return the constructed marker
}

visualization_msgs::MarkerArray visualizer2(std::vector<Eigen::Vector3d>& msg) { // Waypoints
    // Create a Marker for the path points (visualized as spheres)
    visualization_msgs::MarkerArray waypoints;
    for (size_t i = 0; i < msg.size(); ++i)
    {
      visualization_msgs::Marker wpt;
      wpt.header.frame_id = "map"; // Set the frame ID
      wpt.header.stamp = ros::Time::now(); // Set the current time
      wpt.ns = "waypoint"; // Namespace
      wpt.id = i; // Unique ID for this marker
      wpt.type = visualization_msgs::Marker::SPHERE; // Use SPHERE_LIST type
      wpt.action = visualization_msgs::Marker::ADD; // Action to add marker

      wpt.pose.position.x = msg[i].x();
      wpt.pose.position.y = msg[i].y();
      wpt.pose.position.z = msg[i].z();
      wpt.pose.orientation.w = 1.0;

      // Set the scale of the spheres
      wpt.scale.x = 0.3;  // Radius of each sphere
      wpt.scale.y = 0.3;
      wpt.scale.z = 0.3;

      // Set the color of the spheres (green)
      wpt.color.r = 0.0;
      wpt.color.g = 1.0;
      wpt.color.b = 0.0;
      wpt.color.a = 1.0; // Fully opaque
      waypoints.markers.push_back(wpt);
    }
    return waypoints; // Return the constructed marker
}

void generatorThread(ros::Publisher& station_pub, ros::Publisher& target_pub) {
    ros::Rate rate(10);
    
    while (ros::ok()) {
      quadrotor_common::TrajectoryPoint end_point = getFakePoint(3.0);

      if (parallel) {
        visualization_msgs::Marker target_mark = visualizer1(end_point);
        target_pub.publish(target_mark);
      }
      end_point.position.z() = 0.0;
      visualization_msgs::Marker end = visualizer11(end_point); // 노란색 moving object
      station_pub.publish(end);

      if (current_state.mode != "OFFBOARD") { // terminate program if mode is changed
        std::exit(0);
      }

      ros::spinOnce();
      rate.sleep();
    }
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

  // Subscriber

  // SITL
  ros::Subscriber ros_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb); // SITL
  ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, [&state_estimate](const nav_msgs::Odometry::ConstPtr& msg)
                                                                {stateCallback(msg, state_estimate);}
                                                              ); // SITL
  ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback); // SITL
  ros::Subscriber ros_enu_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("gps_vel", 10, enu_vel_cb);
  ros::Subscriber ros_hdg_sub = nh.subscribe<std_msgs::Float64>("compass_hdg", 10, hdg_cb);
  ros::Subscriber hdg_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, hd_cb);
  ros::Subscriber aruco_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_marker/pose", 10, aruco_cb);

  // offboard
  // ros::Subscriber ros_state_sub = nh.subscribe<mavros_msgs::State>("/drone/mavros/state", 10, state_cb);
  // ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("/drone/mavros/local_position/odom", 10, [&state_estimate](const nav_msgs::Odometry::ConstPtr& msg)
  //                                                               {stateCallback(msg, state_estimate);}
  //                                                             );
  // ros::Subscriber ros_enu_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/station/mavros/global_position/raw/gps_vel", 10, enu_vel_cb);
  // ros::Subscriber ros_hdg_sub = nh.subscribe<std_msgs::Float64>("/station/mavros/global_position/compass_hdg", 10, hdg_cb);
  // ros::Subscriber hdg_sub = nh.subscribe<std_msgs::Float64>("/drone/mavros/global_position/compass_hdg", 10, hd_cb);
  
  // ros::Subscriber ros_enu_pos_sub = nh.subscribe<geometry_msgs::Point>("station_enu", 10, enu_pos_cb);
  // ros::Subscriber aruco_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_marker/pose", 10, aruco_cb);
  
  // Publisher
  
  // SITL
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10); // SITL
  
  // offboard
  // ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/drone/mavros/setpoint_velocity/cmd_vel", 10);
  
  //
  // ros::Publisher input_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  ros::Publisher marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
  ros::Publisher marker_array_pub_1 = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array1", 10);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
  ros::Publisher marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker1", 10);
  ros::Publisher reference_pub = nh.advertise<nav_msgs::Path>("reference_trajectory", 1);
  
  double rate = 20.0;
  ros::Rate loop_rate(rate);
  
  PID pid_yaw(1.0, 0.0, 0.1, 1.57);

  // wait until connect to controller
  while(ros::ok() && !current_state.connected){
    ROS_INFO("connecting...");
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  // send a few setpoints before starting to change the mode as OFFBOARD
  geometry_msgs::TwistStamped up_vel;
  up_vel.twist.linear.x = 0.0;
  up_vel.twist.linear.y = 0.0;
  up_vel.twist.linear.z = 1.0;

  // wait until mode change to offboard
  ros::Time up_start_time = ros::Time::now();
  ROS_INFO("Waiting OFFBOARD | current mode: %s", current_state.mode.c_str());
  while (current_state.mode != "OFFBOARD") {
    local_vel_pub.publish(up_vel);
    if ((ros::Time::now() - up_start_time).toSec() > 10.0) {
      ROS_INFO("Time over");
      return 0;
    } else {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  ROS_INFO("MODE is changed to OFFBOARD");

  // Go up to initial height
  // double init_height = 20.0;
  ROS_INFO("Go up to %f m", INIT_HEIGHT);
  while (state_estimate.position.z() < INIT_HEIGHT) {
    local_vel_pub.publish(up_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // start_time = ros::Time::now().toSec();
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "map";
  geometry_msgs::PoseStamped pose;
  std::thread genthread(generatorThread, std::ref(marker_pub_), std::ref(marker_pub));

  quadrotor_common::TrajectoryPoint end_point = getFakePoint(3.0);
  quadrotor_common::TrajectoryPoint reference_point;
  bool is_first = true;

  double criteria = std::sqrt(std::pow(end_point.position.x() - state_estimate.position.x(), 2) + std::pow(end_point.position.y() - state_estimate.position.y(), 2));
  // double vel_gain = 5.0;
  double velocity_gain = vel_gain;
  double look_ahead = std::max(1.0, 0.6*velocity_gain);
  double transition_distance = vel_offset * 3.0;
  while (ros::ok() && criteria > transition_distance) {
    ros::Time time = ros::Time::now();
    path_msg.header.stamp = time;
    end_point = getFakePoint(3.0);

    if (is_first) {
      reference_trajectory = getDynamicReferenceTrajectory(state_estimate, end_point);
      is_first = false;
    } else {
      reference_trajectory = getDynamicReferenceTrajectory2(state_estimate, reference_point, end_point);
    }
    
    // quadrotor_common::TrajectoryPoint end_copy = end_point;
    // end_copy.position.z() = 0.0;
    // visualization_msgs::Marker end = visualizer11(end_copy);
    // marker_pub_.publish(end);

    visualization_msgs::MarkerArray wpt = visualizer2(waypoint);
    marker_array_pub_1.publish(wpt);

    marker_array_pub_.publish(visualizer(path_msg, 0));
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
    visualization_msgs::MarkerArray marker = visualizer(path_msg, 1);
    marker_array_pub_.publish(marker);

    double s_T = ros::Time::now().toSec();
    double c_T = ros::Time::now().toSec();
    while (ros::ok() && c_T - s_T < 1.0) {
      quadrotor_common::TrajectoryPoint e_p = getFakePoint(3.0);
      e_p.position.z() = 0.0;
      visualization_msgs::Marker end = visualizer11(e_p);
      marker_pub_.publish(end);

      reference_point = reference_trajectory.points.front();
      Eigen::Quaterniond q = reference_point.orientation;
      q.normalize();  // Make sure the quaternion is normalized

      // Convert to rotation matrix
      Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

      // Extract Euler angles from rotation matrix (roll, pitch, yaw)
      double roll = std::atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
      double pitch = std::asin(-rotationMatrix(2, 0));
      double yaw = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));

      std::cout << "---" << '\n';
      Eigen::Vector3d pos = reference_point.position;
      Eigen::Vector3d vel = reference_point.velocity;
      double head = reference_point.heading;
      std::cout << "position: " << pos.x() << ", " << pos.y() << ", " << pos.z() << '\n';
      std::cout << "euler   : " << vel.x() << ", " << vel.y() << ", " << vel.z() << '\n';
      std::cout << "velocity: " << roll << ", " << pitch << ", " << yaw << '\n';
      std::cout << "heading : " << head << '\n';
      visualization_msgs::Marker target_mark = visualizer1(reference_point);
      marker_pub.publish(target_mark);

      // double vel_gain = 5.0;
      Eigen::Vector3d vector_current_reference = reference_point.position - state_estimate.position;
      auto& vector = vector_current_reference.normalized();
      auto& vel_vector = vel_gain * vector;

      double target_hdg = getAzimuth(vector.x(), vector.y());
      double current_hdg = hd;
      double delta_yaw = target_hdg - current_hdg;
      if (abs(delta_yaw) > 180) {
        if (delta_yaw > 0) {
          delta_yaw -= 360;
        } else {
          delta_yaw += 360;
        }
      }
      double current_yaw = target_hdg + delta_yaw;
      current_yaw *= (M_PI/180);
      target_hdg *= (M_PI/180);
      std::cout << "curr_yaw: " << current_yaw << '\n';
      std::cout << "target_yaw: " << target_hdg << '\n';

      Eigen::Vector3d target(target_hdg, 0.0, 0.0);
      Eigen::Vector3d current(current_yaw, 0.0, 0.0);
      Eigen::Vector3d yaw_tmp = pid_yaw.calculate(target, current, 0.05);
      double yaw_command = yaw_tmp.x();

      geometry_msgs::TwistStamped vel_com;
      vel_com.twist.linear.x = vel_vector.x();
      vel_com.twist.linear.y = vel_vector.y();
      vel_com.twist.linear.z = vel_vector.z();
      vel_com.twist.angular.z = yaw_command;
      std::cout << "yaw_command: " << yaw_command << '\n';
      double dist_reference = vector_current_reference.norm();
      // std::cout << "state: " << "x: " <<  state_estimate.position.x() << ", y: " <<  state_estimate.position.y() << ", z: " <<  state_estimate.position.z() << '\n';
      // std::cout << "refer: " << "x: " <<  reference_point.position.x() << ", y: " <<  reference_point.position.y() << ", z: " <<  reference_point.position.z() << '\n';
      // std::cout << "dist: " << dist_reference << '\n';

      if (dist_reference < look_ahead) {
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
    criteria = std::sqrt(std::pow(end_point.position.x() - state_estimate.position.x(), 2) + std::pow(end_point.position.y() - state_estimate.position.y(), 2));
  }


  PID pid_xy(1.5, 0.5, 0.1, vel_gain);
  PID pid_z(1.2, 0.0, 0.1, vz_max);

  ROS_INFO("parallel flight");
  geometry_msgs::TwistStamped vel_com;
  vel_com.twist.linear.x = vel_gain;
  vel_com.twist.linear.y = 0.0;
  vel_com.twist.linear.z = 0.0;
  local_vel_pub.publish(vel_com);
  
  double PID_rate = 20.0;
  ros::Rate PID_loop_rate(PID_rate);
  double PID_dt = 1 / PID_rate;
  
  // geometry_msgs::TwistStamped vel_com;
  Eigen::Vector3d target_pos;
  Eigen::Vector3d current_pos;
  Eigen::Vector3d target_z;
  Eigen::Vector3d current_z;
  Eigen::Vector3d vel_vector;
  Eigen::Vector3d vel_z;
  // quadrotor_common::TrajectoryPoint end_point = getFakePoint(3.0);

  double new_criteria = (end_point.position - state_estimate.position).norm();
  double align_thres = 0.1;
  bool loop_active = true;
  bool is_attached = false;  // is satisfied horizontal position threshold
  bool is_satisfied = false;  // duration after satisfied the condition

  bool is_attached2 = false;  // is satisfied aruco horizontal position threshold
  bool is_satisfied2 = false;  // aruco duration after satisfied the condition
  bool first_after_search = true;
  bool is_searching = false;
  bool vertical_flag = false;

  double detected_height;
  ros::param::set("/loop_active", true);
  parallel = true;
  double s_T = ros::Time::now().toSec();
  double c_T = ros::Time::now().toSec();
  // while (new_criteria > align_thres) {
  while (loop_active) {
    end_point = getFakePoint(3.0);
    current_pos = state_estimate.position;

    Eigen::Vector2d station_vec = getENUUnitVector();
    station_vec *= (vel_gain - vel_offset);

    if (is_satisfied) {
      if (is_satisfied2) {
        if (is_detected) {
          target_pos.x() = aruco_pose.pose.position.x;
          target_pos.y() = aruco_pose.pose.position.y;
          // target_pos.z() = aruco_pose.pose.position.z;
        } 
        // else {
        //   std::cout << "ok" << std::endl;
        // }
      } else {
        if (is_detected) {
          target_pos.x() = aruco_pose.pose.position.x;
          target_pos.y() = aruco_pose.pose.position.y;
          if (is_searching) {
            if (first_after_search) {
              ROS_INFO("Marker is detected after going up");
              detected_height = std::round(current_pos.z()) + 1.0;
              first_after_search = false;
            } 
            target_pos.z() = detected_height;
            
            double vertical_criteria = std::fabs(target_pos.z()-current_pos.z());
            double horizontal_criteria = std::sqrt(std::pow(target_pos.x()-current_pos.x(), 2) + std::pow(target_pos.y()-current_pos.y(), 2));
            if (vertical_criteria <= align_thres) {
              if (!vertical_flag) {
                ROS_INFO("Going up is done, horizontal aligning to the marker");                
                vertical_flag = true;
              } else {
                if (horizontal_criteria <= align_thres) {
                  ROS_INFO("Aligning to the marker after search is done");  
                  is_searching = false;
                }
              }
            } else {
              if (!vertical_flag) {
                target_pos.x() = end_point.position.x() - (MARKER_OFFSET*station_vec.x());
                target_pos.y() = end_point.position.y() - (MARKER_OFFSET*station_vec.y());
              }              
            }            
          } else {            
            target_pos.z() = aruco_pose.pose.position.z + HEIGHT_OFFSET;
          }          
        } else {
          if (!is_searching) {
            ROS_INFO("Marker is not detected, going up");
            is_searching = true;
          }
          target_pos.x() = end_point.position.x() - (MARKER_OFFSET*station_vec.x());
          target_pos.y() = end_point.position.y() - (MARKER_OFFSET*station_vec.y());
          target_pos.z() = 4.0;          
        }
      }
    } else {
      target_pos.x() = end_point.position.x() - (MARKER_OFFSET*station_vec.x());
      target_pos.y() = end_point.position.y() - (MARKER_OFFSET*station_vec.y());
      target_pos.z() = end_point.position.z(); 
    }

    new_criteria = (target_pos - current_pos).norm();    

    if (new_criteria <= align_thres && !is_attached) {
      s_T = ros::Time::now().toSec();
      is_attached = true;
      ROS_INFO("Attached to the GPS target point");
    }

    if (is_satisfied && !is_satisfied2) {
      if (is_detected) {
        if (new_criteria <= align_thres && !is_attached2) {          
          is_attached2 = true;
          ROS_INFO("Attached to the marker target point");
          s_T = ros::Time::now().toSec();
        } else if (is_attached2) {
          c_T = ros::Time::now().toSec();
          if (c_T - s_T >= 2) {
            is_satisfied2 = true;
            ROS_INFO("Marker target point condition is satisfied");
          }
        }
      } else {
        c_T = 0;
      }
    }

    target_z.z() = target_pos.z();
    current_z.z() = current_pos.z();

    pid_xy.setVMax(vel_gain); // adaptive to the station velocity
    vel_vector = pid_xy.calculate(target_pos, current_pos, PID_dt);
    vel_z = pid_z.calculate(target_z, current_z, PID_dt);

    double target_hdg;
    if (is_detected && is_satisfied) {
      target_hdg = aruco_pose.pose.orientation.z;
    } else {
      target_hdg = hdg;
    }
    
    double current_hdg = hd;
    double delta_yaw = target_hdg - current_hdg;
    if (abs(delta_yaw) > 180) {
      if (delta_yaw > 0) {
        delta_yaw -= 360;
      } else {
        delta_yaw += 360;
      }
    }
    double current_yaw = target_hdg + delta_yaw;
    current_yaw *= (M_PI/180);
    target_hdg *= (M_PI/180);
    Eigen::Vector3d target(target_hdg, 0.0, 0.0);
    Eigen::Vector3d current(current_yaw, 0.0, 0.0);
    Eigen::Vector3d yaw_tmp = pid_yaw.calculate(target, current, PID_dt);
    double yaw_command = yaw_tmp.x();
    
    // std::cout << "target x: " << target_pos.x() << ", current x: " << current_pos.x() << std::endl;
    // std::cout << "target y: " << target_pos.y() << ", current y: " << current_pos.y() << std::endl;
    // std::cout << "target z: " << 0.0 << ", current z: " << current_pos.z() << std::endl;
    // std::cout << "target yaw: " << target_hdg << ", current yaw: " << current_yaw << std::endl;

    // std::cout << "v_x: " << vel_vector.x() << ", v_y: " << vel_vector.y() << ", v_z: " << vel_z.z() << ", v_yaw: " << yaw_command << std::endl;
    vel_com.twist.linear.x = vel_vector.x();
    vel_com.twist.linear.y = vel_vector.y();
    if (is_attached && !is_satisfied) {
      c_T = ros::Time::now().toSec();
      if (c_T - s_T >= 2) {
        is_satisfied = true;        
        ROS_INFO("GPS target point condition is satisfied");
      }
    }
    
    if (is_satisfied2) {
      vel_com.twist.linear.z = -0.7;
      if (!is_detected) {
        vel_com.twist.linear.x = station_vec.x();
        vel_com.twist.linear.y = station_vec.y();
      }
    } else {
      vel_com.twist.linear.z = vel_z.z();
    }
    vel_com.twist.angular.z = yaw_command;
    
    // std::cout << "speed: " << std::sqrt(std::pow(vel_vector.x(), 2) + std::pow(vel_vector.y(), 2) + std::pow(vel_vector.z(), 2)) << std::endl;
    // std::cout << "distance: " << new_criteria << std::endl;
    
    // std::cout << "aruco_detected: " << is_detected << std::endl;
    // std::cout << "---" << std::endl;
    
    local_vel_pub.publish(vel_com);
    ros::param::get("/loop_active", loop_active);
    ros::spinOnce();
    PID_loop_rate.sleep();
  }

  return 0;
}
