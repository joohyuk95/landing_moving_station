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
#include <boost/bind.hpp>
#include "rpg_mpc/mpc_controller.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>


class MpcController {
public:
  MpcController() {
    state_sub = nh.subscribe("mavros/local_position/odom", 10, stateCallback);
    input_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 10);
  }

  void callMpcRun(quadrotor_common::Trajectory& reference_trajectory) {
    controller.run(quadrotor_common::QuadStateEstimate& current_state, reference_trajectory, rpg_mpc::MpcParams<float>& mpc_params);
  }

  quadrotor_common::Trajectory getReferenceTrajectory() {
    const double max_vel = 2.0;
    const double max_thrust = 15.0;
    const double max_roll_pitch_rate = 0.5;

    quadrotor_common::TrajectoryPoint start_state;
    start_state.position = Eigen::Vector3d(0.0, 0.0, 10.0);
    start_state.heading = 0.0;
    quadrotor_common::TrajectoryPoint end_state;
    end_state.position = Eigen::Vector3d(20.0, 0.0, 0.0);
    end_state.heading = M_PI;

    quadrotor_common::Trajectory fixed_traj = trajectory_generation_helper::polynomials::computeTimeOptimalTrajectory(
      start_state, end_state, 5, max_vel, max_thrust, max_roll_pitch_rate, 50);

    trajectory_generation_helper::heading::addConstantHeadingRate(
      start_state.heading, end_state.heading, &fixed_traj);
    
    return fixed_traj;
  }

  geometry_msgs::PoseStamped command2Attitude(const quadrotor_common::ControlCommand& command) {
    geometry_msgs::PoseStamped attitude;
    attitude.header.stamp = command.timestamp;
    attitude.header.frame_id = "map";

    attitude.pose.orientation.x = command.orientation.x();
    attitude.pose.orientation.y = command.orientation.y();
    attitude.pose.orientation.z = command.orientation.z();
    attitude.pose.orientation.w = command.orientation.w();

    attitude.pose.position.x = 0.0;
    attitude.pose.position.y = 0.0;
    attitude.pose.position.z = 0.0;

    return attitude;
  }

private:
  void stateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_state.timestamp = msg->header.stamp;
    current_state.coordinate_frame = quadrotor_common::QuadStateEstimate::CoordinateFrame::WORLD;

    current_state.position = Eigen::Vector3d(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            msg->pose.pose.position.z);
                                            
    current_state.orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                  msg->pose.pose.orientation.x,
                                                  msg->pose.pose.orientation.y,
                                                  msg->pose.pose.orientation.z);

    current_state.velocity = Eigen::Vector3d(msg->twist.twist.linear.x,
                                          msg->twist.twist.linear.y,
                                          msg->twist.twist.linear.z);

    current_state.bodyrates = Eigen::Vector3d(msg->twist.twist.angular.x,
                                          msg->twist.twist.angular.y,
                                          msg->twist.twist.angular.z);
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  rpg_mpc::MpcController<float> controller(nh, pnh);

  rpg_mpc::MpcParams<float> mpc_params;

  ros::Subscriber state_sub;
  ros::Publisher input_pub;

  quadrotor_common::QuadStateEstimate current_state;
  const ros::Rate loop_rate(50);



} // MpcController class


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rpg_mpc");

  MpcController controller;
  quadrotor_common::Trajectory reference_trajectory = controller.getReferenceTrajectory();

  while (ros::ok() && !reference_trajectory.points.empty()) {
    reference_point = reference_trajectory.points.front();
    
    quadrotor_common::ControlCommand control_command = controller.callMpcRun(reference_point);

    geometry_msgs::PoseStamped setpoint_attitude = controller.command2Attitude(control_command);

    input_pub.publish(setpoint_attitude);

    reference_trajectory.points.pop_front();
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
