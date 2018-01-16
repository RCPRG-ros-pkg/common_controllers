/*
 * Copyright (c) 2010-2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * InterrnalSpaceTrajectoryAction.h
 *
 * Action for both the motor and joint spline interpolation
 *
 *  Created on: 23-09-2010
 *      Author: Konrad Banachowicz, Dawid Seredynski
 */

#ifndef CONTROLLER_COMMON_INTERNAL_SPACE_SPLINE_TRAJECTORY_ACTION_H_
#define CONTROLLER_COMMON_INTERNAL_SPACE_SPLINE_TRAJECTORY_ACTION_H_

#include <string>
#include <vector>
#include <Eigen/Dense>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <actionlib/server/action_server.h>
#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>

#include "rtt_rosclock/rtt_rosclock.h"

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "controller_common/InternalSpaceSplineTrajectory_status.h"

template <class TRAJECTORY_TYPE >
class InternalSpaceSplineTrajectoryAction : public RTT::TaskContext {
 private:
  typedef actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> GoalHandle;
  typedef boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> Goal;

 public:
  explicit InternalSpaceSplineTrajectoryAction(const std::string& name);
  virtual ~InternalSpaceSplineTrajectoryAction();

  bool configureHook();
  bool startHook();
  void updateHook();

 protected:

  typedef Eigen::Matrix<double, TRAJECTORY_TYPE::DOFS, 1> Joints;

  TRAJECTORY_TYPE jnt_command_out_;
  RTT::OutputPort<TRAJECTORY_TYPE > port_jnt_command_out_;

  RTT::InputPort<Joints> port_joint_position_;
  RTT::InputPort<Joints> port_joint_position_command_;
  RTT::InputPort<int32_t> port_generator_status_;

 private:
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);

  void compleatCB();
  void bufferReadyCB();

  std::vector<std::string> jointNames_;

  std::vector<double> lowerLimits_;
  std::vector<double> upperLimits_;

  std::vector<int> remapTable_;

  Joints joint_position_;
  Joints desired_joint_position_;

  // RTT action server
  rtt_actionlib::RTTActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  bool goal_active_;
  GoalHandle activeGoal_;
  bool enable_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;

  Joints stiffness_;

  int cycles_;
};

template <class TRAJECTORY_TYPE >
InternalSpaceSplineTrajectoryAction<TRAJECTORY_TYPE >::InternalSpaceSplineTrajectoryAction(
    const std::string& name)
    : RTT::TaskContext(name, PreOperational)
{
  // Add action server ports to this task's root service
  as_.addPorts(this->provides());

  // Bind action server goal and cancel callbacks (see below)
  as_.registerGoalCallback(
      boost::bind(&InternalSpaceSplineTrajectoryAction<TRAJECTORY_TYPE >::goalCB, this, _1));
  as_.registerCancelCallback(
      boost::bind(&InternalSpaceSplineTrajectoryAction<TRAJECTORY_TYPE >::cancelCB, this, _1));

  this->addPort("jnt_OUTPORT", port_jnt_command_out_);
  this->addPort("JointPosition_INPORT", port_joint_position_);
  this->addPort("JointPositionCommand_INPORT", port_joint_position_command_);
  this->addPort("generator_status_INPORT", port_generator_status_);
  this->addProperty("joint_names", jointNames_);
  this->addProperty("lower_limits", lowerLimits_);
  this->addProperty("upper_limits", upperLimits_);
}

template <class TRAJECTORY_TYPE >
InternalSpaceSplineTrajectoryAction<TRAJECTORY_TYPE >::~InternalSpaceSplineTrajectoryAction() {
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceSplineTrajectoryAction<TRAJECTORY_TYPE >::configureHook() {
  RTT::Logger::In in("InternalSpaceSplineTrajectoryAction::configureHook");

  if (jointNames_.size() != TRAJECTORY_TYPE::DOFS) {
    RTT::log(RTT::Error) << "ROS param joint_names has wrong size:"
                         << jointNames_.size() << ", expected: " << TRAJECTORY_TYPE::DOFS << RTT::endlog();
    return false;
  }

  feedback_.actual.positions.resize(TRAJECTORY_TYPE::DOFS);
  feedback_.desired.positions.resize(TRAJECTORY_TYPE::DOFS);
  feedback_.error.positions.resize(TRAJECTORY_TYPE::DOFS);
  feedback_.joint_names.resize(TRAJECTORY_TYPE::DOFS);

  for (int i = 0; i < jointNames_.size(); i++) {
    feedback_.joint_names.push_back(jointNames_[i]);
  }

  remapTable_.resize(TRAJECTORY_TYPE::DOFS);

  if (lowerLimits_.size() != TRAJECTORY_TYPE::DOFS) {
    RTT::log(RTT::Error) << "ROS param lower_limits has wrong size:"
                         << lowerLimits_.size() << ", expected: " << TRAJECTORY_TYPE::DOFS << RTT::endlog();
    return false;
  }

  if (upperLimits_.size() != TRAJECTORY_TYPE::DOFS) {
    RTT::log(RTT::Error) << "ROS param upper_limits has wrong size:"
                         << upperLimits_.size() << ", expected: " << TRAJECTORY_TYPE::DOFS << RTT::endlog();
    return false;
  }

  stiffness_(0) = 1000;
  for (int i = 1; i < TRAJECTORY_TYPE::DOFS; ++i) {
    stiffness_(i) = 200;
  }

  return true;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceSplineTrajectoryAction<TRAJECTORY_TYPE >::startHook() {
  as_.start();
  goal_active_ = false;
  enable_ = true;

  cycles_ = 0;

  return true;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceSplineTrajectoryAction<TRAJECTORY_TYPE >::updateHook() {
//  bool joint_position_data = true;

//  if (
  port_joint_position_.read(joint_position_);// == RTT::NoData) {
//    joint_position_data = false;
//  }
  control_msgs::FollowJointTrajectoryResult res;

  port_joint_position_command_.read(desired_joint_position_);

  int32_t generator_status;
  if (port_generator_status_.read(generator_status) != RTT::NewData) {
    generator_status = 3;
  }

  if (cycles_ < 100) {
    ++cycles_;
  }

  if (goal_active_ && cycles_ > 2) {
    if (generator_status == 3) {
// TODO: error code is wrong
      res.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      activeGoal_.setAborted(res);
      goal_active_ = false;
      // do nothing
    }
    else if (generator_status == internal_space_spline_trajectory_status::INACTIVE) {
// TODO: error code is wrong
      res.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      activeGoal_.setAborted(res);
      goal_active_ = false;
      // do nothing
    }
    else if (generator_status == internal_space_spline_trajectory_status::ACTIVE) {
      for (int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
        feedback_.actual.positions[i] = joint_position_[i];
        feedback_.desired.positions[i] = desired_joint_position_[i];
        feedback_.error.positions[i] = joint_position_[i]
            - desired_joint_position_[i];
      }

      feedback_.header.stamp = rtt_rosclock::host_now();

      activeGoal_.publishFeedback(feedback_);
    }
    else if (generator_status == internal_space_spline_trajectory_status::SUCCESSFUL) {
      res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      activeGoal_.setSucceeded(res, "");
      goal_active_ = false;
    }
    else if (generator_status == internal_space_spline_trajectory_status::PATH_TOLERANCE_VIOLATED) {
      res.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      activeGoal_.setAborted(res);
      goal_active_ = false;
    }
    else if (generator_status == internal_space_spline_trajectory_status::GOAL_TOLERANCE_VIOLATED) {
      res.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
      activeGoal_.setAborted(res, "");
      goal_active_ = false;
    }
  }
}

template <class TRAJECTORY_TYPE >
void InternalSpaceSplineTrajectoryAction<TRAJECTORY_TYPE >::goalCB(GoalHandle gh) {
  if (!goal_active_) {
    Goal g = gh.getGoal();

    control_msgs::FollowJointTrajectoryResult res;

    RTT::Logger::log(RTT::Logger::Debug) << "Received trajectory contains "
        << g->trajectory.points.size() << " points" << RTT::endlog();

    if (g->trajectory.points.size() > jnt_command_out_.trj.size()) {
        RTT::Logger::log(RTT::Logger::Error)
            << "Trajectory contains too many points" << RTT::endlog();
        res.error_code =
            control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        gh.setRejected(res, "");
        return;
    }


    // fill remap table
    for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
      int jointId = -1;
      for (unsigned int j = 0; j < g->trajectory.joint_names.size(); j++) {
        if (g->trajectory.joint_names[j] == jointNames_[i]) {
          jointId = j;
          break;
        }
      }
      if (jointId < 0) {
        RTT::Logger::log(RTT::Logger::Error)
            << "Trajectory contains invalid joint" << RTT::endlog();
        res.error_code =
            control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        gh.setRejected(res, "");
        return;
      } else {
        remapTable_[i] = jointId;
      }
    }

    // Sprawdzenie ograniczeń w jointach INVALID_GOAL
    bool invalid_goal = false;
    for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
      for (int j = 0; j < g->trajectory.points.size(); j++) {
        if (g->trajectory.points[j].positions[i] > upperLimits_[remapTable_[i]]
            || g->trajectory.points[j].positions[i]
                < lowerLimits_[remapTable_[i]]) {
          RTT::Logger::log(RTT::Logger::Debug) << "Invalid goal [" << i << "]: "
              << upperLimits_[remapTable_[i]] << ">"
              << g->trajectory.points[j].positions[i] << ">"
              << lowerLimits_[remapTable_[i]] << RTT::endlog();
          invalid_goal = true;
        }
      }
    }

    if (invalid_goal) {
      RTT::Logger::log(RTT::Logger::Debug)
          << "Trajectory contains invalid goal!" << RTT::endlog();
      res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      gh.setRejected(res, "");
      goal_active_ = false;
      return;
    }

    jnt_command_out_.start = g->trajectory.header.stamp;

    jnt_command_out_.count_trj = g->trajectory.points.size();

    for (unsigned int i = 0; i < g->trajectory.points.size(); i++) {
      for (unsigned int j = 0; j < g->trajectory.points[i].positions.size();
          j++) {
        jnt_command_out_.trj[i].positions[j] = g->trajectory.points[i].positions[remapTable_[j]];
      }

      for (unsigned int j = 0; j < g->trajectory.points[i].velocities.size();
          j++) {
        jnt_command_out_.trj[i].velocities[j] = g->trajectory.points[i].velocities[remapTable_[j]];
      }

      jnt_command_out_.trj[i].time_from_start = g->trajectory.points[i].time_from_start;
    }

    // prepare tolerances data
    jnt_command_out_.goal_time_tolerance = g->goal_time_tolerance;
    if (g->path_tolerance.size() == g->trajectory.joint_names.size()) {
      for (int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
        jnt_command_out_.path_tolerance[i] = g->path_tolerance[remapTable_[i]].position;
      }
    }
    if (g->goal_tolerance.size() == g->trajectory.joint_names.size()) {
      for (int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
        jnt_command_out_.goal_tolerance[i] = g->goal_tolerance[remapTable_[i]].position;
      }
    }

    // Sprawdzenie czasu w nagłówku OLD_HEADER_TIMESTAMP
    if (g->trajectory.header.stamp < rtt_rosclock::host_now()) {
      RTT::Logger::log(RTT::Logger::Debug) << "Old header timestamp"
          << RTT::endlog();
      res.error_code =
          control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP;
      gh.setRejected(res, "");
      return;
    }

    for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
        jnt_command_out_.stiffness[i] = stiffness_(i);
    }

    activeGoal_ = gh;
    goal_active_ = true;
    cycles_ = 0;

    port_jnt_command_out_.write(jnt_command_out_);

    gh.setAccepted();
    goal_active_ = true;
  } else {
    gh.setRejected();
  }
}

template <class TRAJECTORY_TYPE >
void InternalSpaceSplineTrajectoryAction<TRAJECTORY_TYPE >::cancelCB(GoalHandle gh) {
  goal_active_ = false;
}

#endif  // CONTROLLER_COMMON_INTERNAL_SPACE_SPLINE_TRAJECTORY_ACTION_H_

