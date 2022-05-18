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

#include "fabric_logger/fabric_logger.h"

using fabric_logger::FabricLoggerInterfaceRtPtr;
using fabric_logger::FabricLogger;

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

    VectorNd pos_msr_in_;
    RTT::OutputPort<VectorNd> port_pos_cmd_out_;
    RTT::OutputPort<VectorNd> port_vel_cmd_out_;
    RTT::InputPort<VectorNd> port_pos_msr_in_;
    RTT::InputPort<bool> port_is_synchronised_in_;

    VectorNd stiffness_command_out_;
    RTT::OutputPort<VectorNd> port_stiffness_command_out_;

    int32_t generator_status_;
    RTT::OutputPort<int32_t> port_generator_status_out_;

private:
    void resetTrajectory();
    int getTrajectoryIndex(const ros::Time &now, int trajectory_iteration_index) const;
    int initializeProfile(std::vector<KDL::VelocityProfile_Spline> &vel_profile) const;
    void interpolate(const ros::Time &now, int trajectory_iteration_index, VectorNd& setpoint,
                                                                    VectorNd& vel_setpoint) const;

    bool last_point_not_set_;
    std::vector<KDL::VelocityProfile_Spline> vel_profile_;

    trajectory_msgs::JointTrajectoryPoint trajectory_old_;
    trajectory_msgs::JointTrajectoryPoint trajectory_new_;

    VectorNd des_jnt_pos_, setpoint_, prev_setpoint_, vel_setpoint_, init_trj_point_, init_trj_vel_;

    TRAJECTORY_TYPE trajectory_;

    size_t trajectory_idx_;
    int trajectory_iteration_index_;
    bool empty_trj_received_;
    bool is_profile_initialized_;
    bool first_step_;

    FabricLoggerInterfaceRtPtr m_fabric_logger;
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
    , port_pos_cmd_out_("JointPositionCommand_OUTPORT", true)
    , port_vel_cmd_out_("JointVelocityCommand_OUTPORT", true)
    , port_pos_msr_in_("JointPosition_INPORT")
    , port_is_synchronised_in_("IsSynchronised_INPORT")
    , port_generator_status_out_("generator_status_OUTPORT")
    , port_stiffness_command_out_("stiffness_command_OUTPORT")
    , empty_trj_received_(false)
    , m_fabric_logger( FabricLogger::createNewInterfaceRt( std::string("SplineGen: ") + name, 100000) )
{
    this->ports()->addPort(port_jnt_command_in_);
    this->ports()->addPort(port_pos_cmd_out_);
    this->ports()->addPort(port_vel_cmd_out_);
    this->ports()->addPort(port_pos_msr_in_);
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
int InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::getTrajectoryIndex(
                                    const ros::Time &now, int trajectory_iteration_index) const {
    if (trajectory_.start.isZero()) {
        // Use iteration index
        for (int trajectory_idx_new = trajectory_idx_; trajectory_idx_new < trajectory_.count_trj;
                                                                            trajectory_idx_new++) {
            if (trajectory_.trj[trajectory_idx_new].time_from_start.sec > trajectory_iteration_index) {
                return trajectory_idx_new;
            }
        }
    }
    else {
        // Use time
        for (int trajectory_idx_new = trajectory_idx_; trajectory_idx_new < trajectory_.count_trj;
                                                                              trajectory_idx_new++) {
            ros::Time trj_time = trajectory_.start
                + trajectory_.trj[trajectory_idx_new].time_from_start;
            if (trj_time > now) {
                return trajectory_idx_new;
            }
        }
    }
    return trajectory_.count_trj;
}

template <class TRAJECTORY_TYPE >
int InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::initializeProfile(
                                  std::vector<KDL::VelocityProfile_Spline> &vel_profile) const {
    for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
        double prev_pos, prev_vel, prev_acc;
        double segment_interval;
        bool prev_use_velocities, prev_use_accelerations;
        if (trajectory_idx_ == 0) {
            // init_trj_point_ - use the initial point
            prev_pos = init_trj_point_(i);
            prev_vel = init_trj_vel_(i);
            prev_acc = 0.0;
            prev_use_velocities = true;
            prev_use_accelerations = false;
            segment_interval = trajectory_.trj[trajectory_idx_].time_from_start.toSec();
        }
        else {
            prev_pos = trajectory_.trj[trajectory_idx_ - 1].positions[i];
            prev_vel = trajectory_.trj[trajectory_idx_ - 1].velocities[i];
            prev_acc = trajectory_.trj[trajectory_idx_ - 1].accelerations[i];
            prev_use_velocities = trajectory_.trj[trajectory_idx_ - 1].use_velocities;
            prev_use_accelerations = trajectory_.trj[trajectory_idx_ - 1].use_accelerations;
            segment_interval = (trajectory_.trj[trajectory_idx_].time_from_start
                                - trajectory_.trj[trajectory_idx_ - 1].time_from_start)
                                .toSec();
        }
        double pos = trajectory_.trj[trajectory_idx_].positions[i];
        double vel = trajectory_.trj[trajectory_idx_].velocities[i];
        double acc = trajectory_.trj[trajectory_idx_].accelerations[i];
        bool use_velocities = trajectory_.trj[trajectory_idx_].use_velocities;
        bool use_accelerations = trajectory_.trj[trajectory_idx_].use_accelerations;

        if (prev_use_accelerations && use_accelerations && prev_use_velocities && use_velocities) {
            vel_profile[i].SetProfileDuration( prev_pos, prev_vel, prev_acc,
                                                                pos, vel, acc, segment_interval);
        } else if (prev_use_velocities && use_velocities) {
            vel_profile[i].SetProfileDuration(prev_pos, prev_vel, pos, vel, segment_interval);
        } else {
            vel_profile[i].SetProfileDuration(prev_pos, pos, segment_interval);
        }
    }
}

template <class TRAJECTORY_TYPE >
void InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::interpolate(
                        const ros::Time &now, int trajectory_iteration_index,
                        InternalSpaceSplineTrajectoryGenerator::VectorNd& setpoint,
                        InternalSpaceSplineTrajectoryGenerator::VectorNd& vel_setpoint) const {
    if (trajectory_.start.isZero()) {
        // Use iteration index
        int segment_iteration;
        if (trajectory_idx_ == 0) {
            segment_iteration = trajectory_iteration_index;
        } else {
            segment_iteration = trajectory_iteration_index
                                        - trajectory_.trj[trajectory_idx_ - 1].time_from_start.sec;
        }

        for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
            setpoint(i) = vel_profile_[i].Pos(segment_iteration);
            vel_setpoint(i) = vel_profile_[i].Vel(segment_iteration);
        }
    }
    else {
        // Use time
        double segment_time;
        if (trajectory_idx_ == 0) {
            segment_time = (now - trajectory_.start).toSec();
        } else {
            segment_time = (now - trajectory_.start).toSec()
                - trajectory_.trj[trajectory_idx_ - 1].time_from_start.toSec();
        }

        for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
            setpoint(i) = vel_profile_[i].Pos(segment_time);
            vel_setpoint(i) = vel_profile_[i].Vel(segment_time);
        }
    }
}

template <class TRAJECTORY_TYPE >
void InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::updateHook() {

    if (port_pos_msr_in_.read(pos_msr_in_) != RTT::NewData) {
        m_fabric_logger << "ERROR: could not read port " << port_pos_msr_in_.getName()
                                                                            << FabricLogger::End();
        Logger::In in("InternalSpaceSplineTrajectoryGenerator::updateHook");
        Logger::log() << Logger::Error << "could not read port " << port_pos_msr_in_.getName()
                                                                            << Logger::endl;
        error();
        return;
    }

    //bool read_port_internal_space_position_measurement_in = false;
    if (first_step_) {
        first_step_ = false;
        //FlowStatus read_status = port_pos_msr_in_.read(setpoint_);
        //if (read_status != RTT::NewData) {
        //  Logger::In in("InternalSpaceSplineTrajectoryGenerator::updateHook");
        //  Logger::log() << Logger::Error << "could not read data on port "
        //                << port_pos_msr_in_.getName() << Logger::endl;
        //  error();
        //  return;
        //}
        setpoint_ = pos_msr_in_;

        //for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
        //  vel_setpoint_(i) = 0.0;
        //}
        vel_setpoint_.setZero();

        //read_port_internal_space_position_measurement_in = true;

        bool is_synchronised = true;
        port_is_synchronised_in_.read(is_synchronised);

        if (!is_synchronised) {
            Logger::In in("InternalSpaceSplineTrajectoryGenerator::updateHook");
            Logger::log() << Logger::Error << "not synchronised" << Logger::endl;
            error();
            return;
        }
    }

    if (port_jnt_command_in_.read(jnt_command_in_) == RTT::NewData) {
        // A new trajectory is received
        // TODO: add command checking
        trajectory_ = jnt_command_in_;
        if (trajectory_.count_trj == 0) {
            empty_trj_received_ = true;
        }
        trajectory_idx_ = 0;
        trajectory_iteration_index_ = 0;
        is_profile_initialized_ = false;
        init_trj_point_ = setpoint_;
        init_trj_vel_ = vel_setpoint_;
        prev_setpoint_ = setpoint_;
        last_point_not_set_ = true;
        generator_status_ = internal_space_spline_trajectory_status::ACTIVE;
        for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
            stiffness_command_out_(i) = trajectory_.stiffness[i];
        }
        m_fabric_logger << "received new trajectory" << FabricLogger::End();
    }

    ros::Time now = rtt_rosclock::host_now();
    if (trajectory_idx_ < trajectory_.count_trj && (trajectory_.start < now)) {
        // The trajectory is currently active
        int trajectory_index_new = getTrajectoryIndex(now, trajectory_iteration_index_);
        if (trajectory_idx_ != trajectory_index_new) {
            // A new segment is started
            is_profile_initialized_ = false;
            trajectory_idx_ = trajectory_index_new;
        }

        if (!is_profile_initialized_ && trajectory_idx_ < trajectory_.count_trj) {
            // A profile for the current segment must be initialized
            initializeProfile(vel_profile_);
            is_profile_initialized_ = true;
        }

        if (trajectory_idx_ < trajectory_.count_trj) {
            // There is an active segment
            // Interpolate using current profile
            interpolate(now, trajectory_iteration_index_, setpoint_, vel_setpoint_);

        } else if (last_point_not_set_) {
            // The trajectory is finished, and the last point is not set yet
            for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
                setpoint_(i) = trajectory_.trj[trajectory_.count_trj - 1]
                    .positions[i];
                vel_setpoint_(i) = 0.0;
            }
            last_point_not_set_ = false;
        }

        // Check path tolerance
        for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
            if ( trajectory_.path_tolerance[i] > 0 && fabs(pos_msr_in_(i)-prev_setpoint_(i)) > trajectory_.path_tolerance[i]) {
                resetTrajectory();
                generator_status_ = internal_space_spline_trajectory_status::PATH_TOLERANCE_VIOLATED;
                setpoint_ = pos_msr_in_;
                m_fabric_logger << "path tolerance violated at joint " << i << FabricLogger::End();
                m_fabric_logger << "pos_msr_in: ";
                for (int j = 0; j < TRAJECTORY_TYPE::DOFS; ++j) {
                    m_fabric_logger << pos_msr_in_[j] << ", ";
                }
                m_fabric_logger << FabricLogger::End();

                m_fabric_logger << "setpoint: ";
                for (int j = 0; j < TRAJECTORY_TYPE::DOFS; ++j) {
                    m_fabric_logger << setpoint_[j] << ", ";
                }
                m_fabric_logger << FabricLogger::End();

                m_fabric_logger << "prev_setpoint: ";
                for (int j = 0; j < TRAJECTORY_TYPE::DOFS; ++j) {
                    m_fabric_logger << prev_setpoint_[j] << ", ";
                }
                m_fabric_logger << FabricLogger::End();

                m_fabric_logger << "trajectory_.path_tolerance: ";
                for (int j = 0; j < TRAJECTORY_TYPE::DOFS; ++j) {
                    m_fabric_logger << trajectory_.path_tolerance[j] << ", ";
                }
                m_fabric_logger << FabricLogger::End();
                
                break;
            }
        }

        prev_setpoint_ = setpoint_;
        ++trajectory_iteration_index_;
    }

    if (generator_status_ == internal_space_spline_trajectory_status::ACTIVE) {
        // Check if the goal is reached
        if (empty_trj_received_) {
            resetTrajectory();
            generator_status_ = internal_space_spline_trajectory_status::SUCCESSFUL;
            m_fabric_logger << "the received trajectory is empty" << FabricLogger::End();
        }
        else if (trajectory_idx_ == trajectory_.count_trj) {
            // check goal tolerance
            bool goal_reached = true;
            for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
                if ( trajectory_.goal_tolerance[i] > 0 && fabs(pos_msr_in_(i)-prev_setpoint_(i)) > trajectory_.goal_tolerance[i]) {
                    goal_reached = false;
                    break;
                }
            }

            if (goal_reached) {
                resetTrajectory();
                generator_status_ = internal_space_spline_trajectory_status::SUCCESSFUL;
                m_fabric_logger << "the goal is reached" << FabricLogger::End();
            }
            else {
                if (trajectory_.start.isZero()) {
                    // Use iteration index
                    // goal_time_tolerance is not used in this case
                    resetTrajectory();
                    generator_status_ = internal_space_spline_trajectory_status::GOAL_TOLERANCE_VIOLATED;
                    m_fabric_logger << "goal tolerance violated (iteration index)" << FabricLogger::End();
                }
                else {
                    // Use time
                    ros::Time goal_time = trajectory_.start + trajectory_.trj[trajectory_.count_trj - 1].time_from_start;
                    if (now > goal_time + trajectory_.goal_time_tolerance) {
                        resetTrajectory();
                        generator_status_ = internal_space_spline_trajectory_status::GOAL_TOLERANCE_VIOLATED;
                        m_fabric_logger << "goal tolerance violated (time)" << FabricLogger::End();
                    }
                }
            }
        }
    }
    port_generator_status_out_.write(generator_status_);

    port_stiffness_command_out_.write(stiffness_command_out_);
    port_pos_cmd_out_.write(setpoint_);
    port_vel_cmd_out_.write(vel_setpoint_);

    bool log_all_data = false;
    if (log_all_data) {
      for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
        m_fabric_logger << pos_msr_in_(i) << " ";
      }

      for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
        m_fabric_logger << setpoint_(i) << " ";
      }

      for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
        m_fabric_logger << vel_setpoint_(i) << " ";
      }

      m_fabric_logger << FabricLogger::End();
    }
}

template <class TRAJECTORY_TYPE >
void InternalSpaceSplineTrajectoryGenerator<TRAJECTORY_TYPE >::resetTrajectory() {
    trajectory_idx_ = 0;
    empty_trj_received_ = false;
    trajectory_ = TRAJECTORY_TYPE();
}

#endif  // CONTROLLER_COMMON_INTERNAL_SPACE_SPLINE_TRAJECTORY_GENERATOR_H_

