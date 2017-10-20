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
 * InternalSpaceSplineTrajectoryGenerator.h
 *
 * Generator for both the motor and joint spline interpolation
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz, Dawid Seredynski
 */

#ifndef CONTROLLER_COMMON_INTERNAL_SPACE_SPLINE_TRAJECTORY_GENERATOR_H_
#define CONTROLLER_COMMON_INTERNAL_SPACE_SPLINE_TRAJECTORY_GENERATOR_H_

#include <Eigen/Dense>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include "rtt_rosclock/rtt_rosclock.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <vector>
#include <exception>

#include "controller_common/InternalSpaceSplineTrajectory_status.h"

#include "controller_common/velocityprofile_spline.hpp"

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

template <class TRAJECTORY_TYPE >
class InternalSpaceSplineTrajectoryGenerator : public RTT::TaskContext {
 public:
  explicit InternalSpaceSplineTrajectoryGenerator(const std::string& name);
  virtual ~InternalSpaceSplineTrajectoryGenerator();

  virtual bool configureHook();
  virtual bool startHook();
  virtual void stopHook();
  virtual void updateHook();

 protected:
  typedef Eigen::Matrix<double, TRAJECTORY_TYPE::DOFS, 1>  VectorNd;

  TRAJECTORY_TYPE jnt_command_in_;
  RTT::InputPort<TRAJECTORY_TYPE > port_jnt_command_in_;

  VectorNd internal_space_position_measurement_in_;
  RTT::OutputPort<VectorNd> port_internal_space_position_command_out_;
  RTT::InputPort<VectorNd> port_internal_space_position_measurement_in_;
  RTT::InputPort<bool> port_is_synchronised_in_;

  VectorNd stiffness_command_out_;
  RTT::OutputPort<VectorNd> port_stiffness_command_out_;

  int32_t generator_status_;
  RTT::OutputPort<int32_t> port_generator_status_out_;

 private:
  void resetTrajectory();

  bool last_point_not_set_;
  std::vector<KDL::VelocityProfile_Spline> vel_profile_;

  trajectory_msgs::JointTrajectoryPoint trajectory_old_;
  trajectory_msgs::JointTrajectoryPoint trajectory_new_;

  VectorNd des_jnt_pos_, setpoint_, prev_setpoint_, old_point_;

  TRAJECTORY_TYPE trajectory_;

  size_t trajectory_idx_;
  bool empty_trj_received_;

  bool first_step_;
};

using namespace RTT;

template <class TRAJECTORY_TYPE >
InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::InternalSpaceSplineTrajectoryGenerator(
    const std::string& name)
    : RTT::TaskContext(name, PreOperational)
    , last_point_not_set_(false)
    , trajectory_idx_(0)
    , trajectory_(TRAJECTORY_TYPE())
    , port_jnt_command_in_("jnt_INPORT")
    , port_internal_space_position_command_out_("JointPositionCommand_OUTPORT", true)
    , port_internal_space_position_measurement_in_("JointPosition_INPORT")
    , port_is_synchronised_in_("IsSynchronised_INPORT")
    , port_generator_status_out_("generator_status_OUTPORT")
    , port_stiffness_command_out_("stiffness_command_OUTPORT")
    , empty_trj_received_(false) {
  this->ports()->addPort(port_jnt_command_in_);
  this->ports()->addPort(port_internal_space_position_command_out_);
  this->ports()->addPort(port_internal_space_position_measurement_in_);
  this->ports()->addPort(port_is_synchronised_in_);
  this->ports()->addPort(port_generator_status_out_);
  this->ports()->addPort(port_stiffness_command_out_);

  return;
}

template <class TRAJECTORY_TYPE >
InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::~InternalSpaceSplineTrajectoryGenerator() {
  return;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::configureHook() {
  Logger::In in("InternalSpaceSplineTrajectoryGenerator::configureHook");

  vel_profile_.resize(TRAJECTORY_TYPE::DOFS);

  return true;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::startHook() {
  RESTRICT_ALLOC;

  first_step_ = true;

  last_point_not_set_ = false;

  resetTrajectory();

  generator_status_ = internal_space_spline_trajectory_status::INACTIVE;

  return true;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::stopHook() {
  UNRESTRICT_ALLOC;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::updateHook() {

  bool read_port_internal_space_position_measurement_in = false;
  if (first_step_) {
    FlowStatus read_status = port_internal_space_position_measurement_in_.read(setpoint_);
    if (read_status != RTT::NewData) {
      Logger::In in("InternalSpaceSplineTrajectoryGenerator::updateHook");
      Logger::log() << Logger::Error << "could not read data on port "
                    << port_internal_space_position_measurement_in_.getName() << Logger::endl;
      error();
      return;
    }

    read_port_internal_space_position_measurement_in = true;

    bool is_synchronised = true;
    port_is_synchronised_in_.read(is_synchronised);

    if (!is_synchronised) {
      Logger::In in("InternalSpaceSplineTrajectoryGenerator::updateHook");
      Logger::log() << Logger::Error << "not synchronised" << Logger::endl;
      error();
      return;
    }
    first_step_ = false;
  }

    if (port_jnt_command_in_.read(jnt_command_in_) == RTT::NewData) {
// TODO: add command checking
        trajectory_ = jnt_command_in_;
        if (trajectory_.count_trj == 0) {
            empty_trj_received_ = true;
        }
        trajectory_idx_ = 0;
        old_point_ = setpoint_;
        prev_setpoint_ = setpoint_;
        last_point_not_set_ = true;
        generator_status_ = internal_space_spline_trajectory_status::ACTIVE;
        for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
            stiffness_command_out_(i) = trajectory_.stiffness[i];
        }
    }

    ros::Time now = rtt_rosclock::host_now();
    if (trajectory_idx_ < trajectory_.count_trj && (trajectory_.start < now)) {
        for (; trajectory_idx_ < trajectory_.count_trj; trajectory_idx_++) {
            ros::Time trj_time = trajectory_.start
                + trajectory_.trj[trajectory_idx_].time_from_start;
            if (trj_time > now) {
                for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
                    if (trajectory_idx_ < 1) {
                        if (trajectory_.trj[trajectory_idx_].use_accelerations
                              && trajectory_.trj[trajectory_idx_].use_velocities) {
                            vel_profile_[i].SetProfileDuration(
                                old_point_(i), 0.0, 0.0,
                                trajectory_.trj[trajectory_idx_].positions[i],
                                trajectory_.trj[trajectory_idx_].velocities[i],
                                trajectory_.trj[trajectory_idx_].accelerations[i],
                                trajectory_.trj[trajectory_idx_].time_from_start.toSec());
                        } else if (trajectory_.trj[trajectory_idx_].use_velocities) {
                            vel_profile_[i].SetProfileDuration(
                                old_point_(i), 0.0,
                                trajectory_.trj[trajectory_idx_].positions[i],
                                trajectory_.trj[trajectory_idx_].velocities[i],
                                trajectory_.trj[trajectory_idx_].time_from_start.toSec());
                        } else {
                            vel_profile_[i].SetProfileDuration(
                                old_point_(i),
                                trajectory_.trj[trajectory_idx_].positions[i],
                                trajectory_.trj[trajectory_idx_].time_from_start.toSec());
                          }
                      } else {
                        if (trajectory_.trj[trajectory_idx_ - 1].use_accelerations
                              && trajectory_.trj[trajectory_idx_].use_accelerations
                              && trajectory_.trj[trajectory_idx_ - 1].use_velocities
                              && trajectory_.trj[trajectory_idx_].use_velocities) {
                            vel_profile_[i].SetProfileDuration(
                                trajectory_.trj[trajectory_idx_ - 1].positions[i],
                                trajectory_.trj[trajectory_idx_ - 1].velocities[i],
                                trajectory_.trj[trajectory_idx_ - 1].accelerations[i],
                                trajectory_.trj[trajectory_idx_].positions[i],
                                trajectory_.trj[trajectory_idx_].velocities[i],
                                trajectory_.trj[trajectory_idx_].accelerations[i],
                                (trajectory_.trj[trajectory_idx_].time_from_start
                                    - trajectory_.trj[trajectory_idx_ - 1].time_from_start)
                                    .toSec());
                        } else if (trajectory_.trj[trajectory_idx_ - 1].use_velocities
                              && trajectory_.trj[trajectory_idx_].use_velocities) {
                            vel_profile_[i].SetProfileDuration(
                                trajectory_.trj[trajectory_idx_ - 1].positions[i],
                                trajectory_.trj[trajectory_idx_ - 1].velocities[i],
                                trajectory_.trj[trajectory_idx_].positions[i],
                                trajectory_.trj[trajectory_idx_].velocities[i],
                                (trajectory_.trj[trajectory_idx_].time_from_start
                                    - trajectory_.trj[trajectory_idx_ - 1].time_from_start)
                                    .toSec());
                        } else {
                            vel_profile_[i].SetProfileDuration(
                                trajectory_.trj[trajectory_idx_ - 1].positions[i],
                                trajectory_.trj[trajectory_idx_].positions[i],
                                (trajectory_.trj[trajectory_idx_].time_from_start
                                    - trajectory_.trj[trajectory_idx_ - 1].time_from_start)
                                    .toSec());
                        }
                    }
                }
                break;
            }
        }

        if (read_port_internal_space_position_measurement_in) {
            internal_space_position_measurement_in_ = setpoint_;
        }
        else {
            if (port_internal_space_position_measurement_in_.read(internal_space_position_measurement_in_) != RTT::NewData) {
                  Logger::In in("InternalSpaceSplineTrajectoryGenerator::updateHook");
                  Logger::log() << Logger::Error << "could not read port " << port_internal_space_position_measurement_in_.getName() << Logger::endl;
                error();
                return;
            }
        }

        if (trajectory_idx_ < trajectory_.count_trj) {
            double t;
            if (trajectory_idx_ < 1) {
                t = (now - trajectory_.start).toSec();
            } else {
                t = (now - trajectory_.start).toSec()
                    - trajectory_.trj[trajectory_idx_ - 1].time_from_start.toSec();
            }

            for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
                setpoint_(i) = vel_profile_[i].Pos(t);
            }

        } else if (last_point_not_set_) {
            for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
                setpoint_(i) = trajectory_.trj[trajectory_.count_trj - 1]
                    .positions[i];
            }
            last_point_not_set_ = false;
        }

        // check path tolerance
        for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
            if ( trajectory_.path_tolerance[i] > 0 && fabs(internal_space_position_measurement_in_(i)-prev_setpoint_(i)) > trajectory_.path_tolerance[i]) {
                resetTrajectory();
                generator_status_ = internal_space_spline_trajectory_status::PATH_TOLERANCE_VIOLATED;
            }
        }

        prev_setpoint_ = setpoint_;
    }

    if (trajectory_idx_ > 0 && trajectory_idx_ == trajectory_.count_trj) {
        ros::Time goal_time = trajectory_.start + trajectory_.trj[trajectory_.count_trj - 1].time_from_start;
        // check goal tolerance
        bool goal_reached = true;
        for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
            if ( trajectory_.goal_tolerance[i] > 0 && fabs(internal_space_position_measurement_in_(i)-prev_setpoint_(i)) > trajectory_.goal_tolerance[i]) {
                goal_reached = false;
            }
        }

        if (now > goal_time + trajectory_.goal_time_tolerance) {
            if (goal_reached) {
                resetTrajectory();
                generator_status_ = internal_space_spline_trajectory_status::SUCCESSFUL;
            }
            else {
                resetTrajectory();
                generator_status_ = internal_space_spline_trajectory_status::GOAL_TOLERANCE_VIOLATED;
            }
        }
        else if (now > goal_time - trajectory_.goal_time_tolerance) {
            if (goal_reached) {
                resetTrajectory();
                generator_status_ = internal_space_spline_trajectory_status::SUCCESSFUL;
            }
        }
    }
    else if (empty_trj_received_) {
        resetTrajectory();
        generator_status_ = internal_space_spline_trajectory_status::SUCCESSFUL;
    }
    port_generator_status_out_.write(generator_status_);

    port_stiffness_command_out_.write(stiffness_command_out_);
    port_internal_space_position_command_out_.write(setpoint_);
}

template <class TRAJECTORY_TYPE >
void InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::resetTrajectory() {
    trajectory_idx_ = 0;
    empty_trj_received_ = false;
    trajectory_ = TRAJECTORY_TYPE();
}

#endif  // CONTROLLER_COMMON_INTERNAL_SPACE_SPLINE_TRAJECTORY_GENERATOR_H_

