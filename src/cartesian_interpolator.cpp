// Copyright 2014 WUT
/*
 * CartesianInterpolator.cpp
 *
 *  Created on: 27 lut 2014
 *      Author: konradb3
 */

#include "cartesian_interpolator.h"

#include <string>

#include "rtt_rosclock/rtt_rosclock.h"
#include "Eigen/Geometry"

CartesianInterpolator::CartesianInterpolator(const std::string& name)
    : RTT::TaskContext(name),
      trajectory_ptr_(0),
      activate_pose_init_property_(false),
      last_point_not_set_(false),
      trajectory_active_(false),
      ns_interval_(0) {
  this->ports()->addPort("CartesianPosition", port_cartesian_position_);
  this->ports()->addPort("CartesianPositionCommand", port_cartesian_command_);
  this->ports()->addPort("CartesianTrajectoryCommand", port_trajectory_);
  this->ports()->addPort("GeneratorActiveOut", port_generator_active_);
  this->ports()->addPort("IsSynchronisedIn", port_is_synchronised_);

  this->addProperty("activate_pose_init", activate_pose_init_property_);
  this->addProperty("init_setpoint", init_setpoint_property_);
  this->addProperty("ns_interval", ns_interval_);
}

CartesianInterpolator::~CartesianInterpolator() {
}

bool CartesianInterpolator::configureHook() {
  ns_higher_bound_ = ns_interval_ * 1.1;
  ns_higher_increment_ = ns_interval_ * 1.05;
  ns_lower_bound_ = ns_interval_ * 0.9;
  ns_lower_increment_ = ns_interval_ * 0.95;

  return true;
}

bool CartesianInterpolator::startHook() {
  bool is_synchronised = true;
  if (activate_pose_init_property_) {
    setpoint_ = init_setpoint_property_;
  } else {
    if (port_cartesian_position_.read(setpoint_) == RTT::NoData) {
      return false;
    }
  }

  port_is_synchronised_.read(is_synchronised);

  if (!is_synchronised) {
    return false;
  }
  port_generator_active_.write(true);
  last_point_not_set_ = false;
  trajectory_active_ = false;

  last_time_ = rtt_rosclock::host_now();
  update_hook_iter_ = 0;

  return true;
}

void CartesianInterpolator::stopHook() {
  port_generator_active_.write(false);
}

void CartesianInterpolator::updateHook() {
  update_hook_iter_++;
  port_generator_active_.write(true);
  if (port_trajectory_.read(trajectory_) == RTT::NewData) {
    trajectory_ptr_ = 0;
    old_point_ = setpoint_;
    last_point_not_set_ = true;
    trajectory_active_ = true;
  }

  ros::Time now = rtt_rosclock::host_now();
  if ((ns_higher_bound_ > 0)
      && (now - last_time_) >= ros::Duration(0, ns_higher_bound_)) {
    // std::cout << now - last_time_ << std::endl;
    last_time_ += ros::Duration(0, ns_higher_increment_);
    now = last_time_;
  } else if ((ns_lower_bound_ > 0)
      && ((now - last_time_) <= ros::Duration(0, ns_lower_bound_))
      && (update_hook_iter_ > 1)) {
    // std::cout << now - last_time_ << std::endl;
    last_time_ += ros::Duration(0, ns_lower_increment_);
    now = last_time_;
  }
  last_time_ = now;

  if (trajectory_active_ && trajectory_ && (trajectory_->header.stamp < now)) {
    for (; trajectory_ptr_ < trajectory_->points.size(); trajectory_ptr_++) {
      ros::Time trj_time = trajectory_->header.stamp
          + trajectory_->points[trajectory_ptr_].time_from_start;
      if (trj_time > now) {
        break;
      }
    }

    if (trajectory_ptr_ < trajectory_->points.size()) {
      if (trajectory_ptr_ == 0) {
        cartesian_trajectory_msgs::CartesianTrajectoryPoint p0;
        p0.time_from_start.fromSec(0.0);
        p0.pose = old_point_;
        setpoint_ = interpolate(p0, trajectory_->points[trajectory_ptr_], now);
      } else {
        setpoint_ = interpolate(trajectory_->points[trajectory_ptr_ - 1],
                                trajectory_->points[trajectory_ptr_], now);
      }
    } else if (last_point_not_set_) {
      setpoint_ = trajectory_->points[trajectory_->points.size() - 1].pose;
      last_point_not_set_ = false;
    }
  }
  port_cartesian_command_.write(setpoint_);
}

geometry_msgs::Pose CartesianInterpolator::interpolate(
    const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p0,
    const cartesian_trajectory_msgs::CartesianTrajectoryPoint& p1,
    ros::Time t) {
  geometry_msgs::Pose pose;

  ros::Time t0 = trajectory_->header.stamp + p0.time_from_start;
  ros::Time t1 = trajectory_->header.stamp + p1.time_from_start;

  pose.position.x = interpolate(p0.pose.position.x, p1.pose.position.x,
                                t0.toSec(), t1.toSec(), t.toSec());
  pose.position.y = interpolate(p0.pose.position.y, p1.pose.position.y,
                                t0.toSec(), t1.toSec(), t.toSec());
  pose.position.z = interpolate(p0.pose.position.z, p1.pose.position.z,
                                t0.toSec(), t1.toSec(), t.toSec());

  Eigen::Quaterniond q0(p0.pose.orientation.w, p0.pose.orientation.x,
                        p0.pose.orientation.y, p0.pose.orientation.z);
  Eigen::Quaterniond q1(p1.pose.orientation.w, p1.pose.orientation.x,
                        p1.pose.orientation.y, p1.pose.orientation.z);

  double a = interpolate(0.0, 1.0, t0.toSec(), t1.toSec(), t.toSec());
  Eigen::Quaterniond q = q0.slerp(a, q1);
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();

  return pose;
}

double CartesianInterpolator::interpolate(double p0, double p1, double t0,
                                          double t1, double t) {
  return (p0 + (p1 - p0) * (t - t0) / (t1 - t0));
}
