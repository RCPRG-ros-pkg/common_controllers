// Copyright 2014 WUT
/*
 * CartesianInterpolator.h
 *
 *  Created on: 27 lut 2014
 *      Author: konradb3
 */

#ifndef CARTESIAN_INTERPOLATOR_H_
#define CARTESIAN_INTERPOLATOR_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "cartesian_trajectory_msgs/CartesianTrajectory.h"
#include "geometry_msgs/Pose.h"

#include "Eigen/Dense"

class CartesianInterpolator : public RTT::TaskContext {
 public:
  explicit CartesianInterpolator(const std::string& name);
  virtual ~CartesianInterpolator();
  virtual bool configureHook();
  virtual bool startHook();
  virtual void stopHook();
  virtual void updateHook();

 private:
  geometry_msgs::Pose interpolate(
      const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p0,
      const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p1,
      ros::Time t);
  double interpolate(double p0, double p1, double t0, double t1, double t);
  RTT::InputPort<cartesian_trajectory_msgs::CartesianTrajectoryConstPtr> port_trajectory_;
  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_;

  RTT::OutputPort<geometry_msgs::Pose> port_cartesian_command_;
  RTT::OutputPort<bool> port_generator_active_;
  RTT::InputPort<bool> port_is_synchronised_;

  cartesian_trajectory_msgs::CartesianTrajectoryConstPtr trajectory_;
  geometry_msgs::Pose setpoint_;
  geometry_msgs::Pose old_point_;

  size_t trajectory_ptr_;

  bool activate_pose_init_property_;
  geometry_msgs::Pose init_setpoint_property_;

  bool last_point_not_set_;
  bool trajectory_active_;

  ros::Time last_time_;
  int update_hook_iter_;

  int ns_higher_bound_;
  int ns_higher_increment_;
  int ns_lower_bound_;
  int ns_lower_increment_;

  // properties
  int ns_interval_;
};

#endif  // CARTESIAN_INTERPOLATOR_H_
