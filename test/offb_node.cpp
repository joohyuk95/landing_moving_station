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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
// #include "ros/ros.h"
// #include "ros/console.h"
// #include "rpg_mpc/mpc_controller.h"
// #include <nav_msgs/Odometry.h>
// #include <mavros_msgs/AttitudeTarget.h>
// #include <mavros_msgs/State.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <polynomial_trajectories/polynomial_trajectory_settings.h>
// #include <trajectory_generation_helper/heading_trajectory_helper.h>
// #include <trajectory_generation_helper/polynomial_trajectory_helper.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    std::cout << "state_cb" << '\n';
    current_state = *msg;
}

// void stateCallback(const nav_msgs::Odometry::ConstPtr& msg, quadrotor_common::QuadStateEstimate& state_estimate) {
//   ROS_INFO("statecallback");
//   state_estimate.timestamp = msg->header.stamp;
//   state_estimate.coordinate_frame = quadrotor_common::QuadStateEstimate::CoordinateFrame::WORLD;

//   state_estimate.position = Eigen::Vector3d(msg->pose.pose.position.x,
//                                            msg->pose.pose.position.y,
//                                            msg->pose.pose.position.z);
                                          
//   state_estimate.orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
//                                                  msg->pose.pose.orientation.x,
//                                                  msg->pose.pose.orientation.y,
//                                                  msg->pose.pose.orientation.z);

//   state_estimate.velocity = Eigen::Vector3d(msg->twist.twist.linear.x,
//                                            msg->twist.twist.linear.y,
//                                            msg->twist.twist.linear.z);

//   state_estimate.bodyrates = Eigen::Vector3d(msg->twist.twist.angular.x,
//                                             msg->twist.twist.angular.y,
//                                             msg->twist.twist.angular.z);
// }

// quadrotor_common::Trajectory getReferenceTrajectory() {
//   const double max_vel = 2.0;
//   const double max_thrust = 15.0;
//   const double max_roll_pitch_rate = 0.5;

//   quadrotor_common::TrajectoryPoint start_state;
//   start_state.position = Eigen::Vector3d(0.0, 0.0, 10.0);
//   start_state.heading = 0.0;
//   quadrotor_common::TrajectoryPoint end_state;
//   end_state.position = Eigen::Vector3d(20.0, 0.0, 0.0);
//   end_state.heading = M_PI;

//   quadrotor_common::Trajectory fixed_traj = trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
//     start_state, end_state, 5, max_vel, max_thrust, max_roll_pitch_rate, 50);

//   trajectory_generation_helper::heading::addConstantHeadingRate(
//     start_state.heading, end_state.heading, &fixed_traj);
  
//   return fixed_traj;
// }

// mavros_msgs::AttitudeTarget command2Bodyrate(const quadrotor_common::ControlCommand& command) {
//   quadrotor_msgs::ControlCommand ros_command = command.toRosMessage();
  
//   mavros_msgs::AttitudeTarget attitude;
  
//   attitude.header.stamp = ros_command.header.stamp;
//   attitude.header.frame_id = "map";

//   attitude.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

//   attitude.orientation.w = 1.0;

//   attitude.body_rate.x = ros_command.bodyrates.x;
//   attitude.body_rate.y = ros_command.bodyrates.y;
//   attitude.body_rate.z = ros_command.bodyrates.z;

//   attitude.thrust = ros_command.collective_thrust;

//   return attitude;
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;
  // ros::NodeHandle pnh("~");
  // rpg_mpc::MpcController<float> controller(nh, pnh);

  // quadrotor_common::QuadStateEstimate state_estimate;
  // quadrotor_common::Trajectory reference_trajectory;
  // rpg_mpc::MpcParams<float> mpc_params;

  ros::Subscriber ros_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  // ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, [&state_estimate](const nav_msgs::Odometry::ConstPtr& msg)
  //                                                               {stateCallback(msg, state_estimate);
  //                                                             });
  // ros::Publisher input_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Rate loop_rate(20.0);
  
  while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
        std::cout << "spin" << '\n';
  }

  // reference_trajectory = getReferenceTrajectory();
  // std::cout << "trajectory points" << reference_trajectory.points.begin() << std::endl;
  // for (const auto& point: reference_trajectory.points) {
  //   std::cout << "x: " << point.position.x()
  //             << "  y: " << point.position.y()
  //             << "  z: " << point.position.z()
  //             << "  head: " << point.heading
  //             << std::endl;
  // }

  // for (int i = 0; i < reference_trajectory.points.size(); ++i) {
  //   const auto& first_point = reference_trajectory.points.front();
  //   std::cout << "x: " << first_point.position.x()
  //           << "  y: " << first_point.position.y()
  //           << "  z: " << first_point.position.z()
  //           << std::endl;
  //   reference_trajectory.points.pop_front();
  // }
  // const auto& first_point = reference_trajectory.points.front();
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  // while (ros::ok() && !reference_trajectory.points.empty()) {
  while(ros::ok()){
    // quadrotor_common::TrajectoryPoint reference_point = reference_trajectory.points.front();
    
    // quadrotor_common::ControlCommand control_command = controller.run(state_estimate, reference_point, mpc_params);
    
    // mavros_msgs::AttitudeTarget setpoint_attitude = command2Bodyrate(control_command);

    // std::cout << "Orientation: [" 
    //       << setpoint_attitude.orientation.x << ", " 
    //       << setpoint_attitude.orientation.y << ", " 
    //       << setpoint_attitude.orientation.z << ", " 
    //       << setpoint_attitude.orientation.w << "]" << std::endl;
    // std::cout << "Thrust: " << setpoint_attitude.thrust << std::endl;
    // std::cout << "set position: " << pose << std::endl;
    // input_pub.publish(setpoint_attitude);
    // local_pos_pub.publish(pose);
    // std::cout << "publish" << std::endl;
    // reference_trajectory.points.pop_front();
    
    ros::spinOnce();
    
    loop_rate.sleep();
  }

  return 0;
}
