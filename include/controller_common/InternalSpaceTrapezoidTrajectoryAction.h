/*
 * Copyright (c) 2010-2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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
 * InterrnalSpaceTrapezoidTrajectoryAction.h
 *
 * Action for both the motor and joint trapezoid interpolation
 *
 *  Created on: 23-09-2010
 *      Author: Konrad Banachowicz, Dawid Seredynski, Karolina Borkowska
 */

#ifndef CONTROLLER_COMMON_INTERNAL_SPACE_TRAPEZOID_TRAJECTORY_ACTION_H_
#define CONTROLLER_COMMON_INTERNAL_SPACE_TRAPEZOID_TRAJECTORY_ACTION_H_
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>

#include "rtt_rosclock/rtt_rosclock.h"

//#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <trapezoid_trajectory_msgs/TrapezoidTrajectoryAction.h>
//#include <rtt/Component.hpp>

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

template <class TRAJECTORY_TYPE >
class InternalSpaceTrapezoidTrajectoryAction : public RTT::TaskContext {
 private:
  typedef actionlib::ServerGoalHandle<trapezoid_trajectory_msgs::TrapezoidTrajectoryAction> GoalHandle;
  typedef boost::shared_ptr<const trapezoid_trajectory_msgs::TrapezoidTrajectoryGoal> Goal;

 public:
  explicit InternalSpaceTrapezoidTrajectoryAction(const std::string& name);
  virtual ~InternalSpaceTrapezoidTrajectoryAction();

  bool configureHook();
  bool startHook();
  void updateHook();

 protected:
  TRAJECTORY_TYPE jnt_command_out_;
  typedef Eigen::Matrix<double, TRAJECTORY_TYPE::DOFS, 1> Joints;

  //communication with api
  //RTT::InputPort<trapezoid_trajectory_msgs::TrapezoidTrajectoryGoal> port_jnt_command_port_;

  //communication with generator
  RTT::OutputPort<TRAJECTORY_TYPE> port_jnt_command_out_;
  RTT::InputPort<int32_t> port_generator_status_;

  //feedback information
  RTT::InputPort<Joints> port_joint_position_;
  RTT::InputPort<Joints> port_joint_position_command_;

 private:
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);

  void compleatCB();
  void bufferReadyCB();

  bool remapJointsAndLimits(
      trajectory_msgs::JointTrajectory* trj_ptr, Goal g, double* max_vel, double* max_acc);
  bool trajectoryHasInvalidPoints(Goal g);
  bool fillRemapTable(Goal g);
  bool goalToleranceIsViolated(Goal g);
  bool pathToleranceBreached(Goal g);
  bool getPeersReady();

  //std::vector<KDL::VelocityProfile_Spline> vel_profile_;

  std::vector<std::string> jointNames_;

  std::vector<double> lowerLimits_;
  std::vector<double> upperLimits_;

  std::vector<int> remapTable_;

  Joints joint_position_;
  Joints desired_joint_position_;
  Joints stiffness_;

  // RTT action server
  rtt_actionlib::RTTActionServer<trapezoid_trajectory_msgs::TrapezoidTrajectoryAction> as_;

  GoalHandle activeGoal_;
  bool goal_active_;

  trapezoid_trajectory_msgs::TrapezoidTrajectoryFeedback feedback_;

  int cycles_;
};

template <class TRAJECTORY_TYPE >
InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::InternalSpaceTrapezoidTrajectoryAction(
    const std::string& name)
    : RTT::TaskContext(name, PreOperational)
{
  // Add action server ports to this task's root service
  as_.addPorts(this->provides());

  // Bind action server goal and cancel callbacks (see below)
  as_.registerGoalCallback(
      boost::bind(&InternalSpaceTrapezoidTrajectoryAction::goalCB, this, _1));
  as_.registerCancelCallback(
      boost::bind(&InternalSpaceTrapezoidTrajectoryAction::cancelCB, this, _1));

  this->addPort("jnt_OUTPORT", port_jnt_command_out_);
  this->addPort("JointPosition_INPORT", port_joint_position_);
  this->addPort("JointPositionCommand_INPORT", port_joint_position_command_);
  this->addPort("generator_status_INPORT", port_generator_status_);
  this->addProperty("joint_names", jointNames_);
  this->addProperty("lower_limits", lowerLimits_);
  this->addProperty("upper_limits", upperLimits_);
}

template <class TRAJECTORY_TYPE >
InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::~InternalSpaceTrapezoidTrajectoryAction() {
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::configureHook() {
// configuration of a hook that is called in updateHook every
// time quant given in irp6_ui/irp6_bringup/config/irp6-p-inside.ops
// currently (13.11.2017) set to 0.01 sec (100Hz)
  RTT::Logger::In in("InternalSpaceTrapezoidTrajectoryAction::configureHook");

  //check if there are any joints asigned
  if (jointNames_.size() != TRAJECTORY_TYPE::DOFS) {
    RTT::log(RTT::Error) << "ROS param joint_names has wrong size:"
                         << jointNames_.size() << ", expected: " << TRAJECTORY_TYPE::DOFS << RTT::endlog();
    return false;
  }

  //trapezoid_trajectory_msgs::TrapezoidTrajectoryFeedback feedback_
  //describes parameters for comparison
  feedback_.actual.positions.resize(TRAJECTORY_TYPE::DOFS);
  feedback_.desired.positions.resize(TRAJECTORY_TYPE::DOFS);
  feedback_.error.positions.resize(TRAJECTORY_TYPE::DOFS);
  feedback_.joint_names.resize(TRAJECTORY_TYPE::DOFS);


  //adds names of joints to feedback_ table
  for (int i = 0; i < jointNames_.size(); i++) {
    feedback_.joint_names.push_back(jointNames_[i]);
  }

  //remapTable is used to remap joints between a given goal table
  //and corresponding spots in jointNames_ table (in goalCB)
  remapTable_.resize(TRAJECTORY_TYPE::DOFS);

  //checks wether limits have been loaded properly
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
bool InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE >::startHook() {
  as_.start();
  goal_active_ = false;

  cycles_ = 0;

  return true;
}


template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::updateHook() {
  //checking for joint position for feedback publishing
  port_joint_position_.read(joint_position_);

  if (cycles_ < 100) {
    ++cycles_;
  }

  //what the generator has given as a setpoint for this iteration
  port_joint_position_command_.read(desired_joint_position_);

  //end goal
  Goal g = activeGoal_.getGoal();

  //msgs for comunication with generator
  trapezoid_trajectory_msgs::TrapezoidTrajectoryResult res = trapezoid_trajectory_msgs::TrapezoidTrajectoryResult();
  int32_t genMsg;

  // if the goal is still unreached and the component was able to
  // recieve data about joint position feedback is send to irpos api
  if (goal_active_ && cycles_ > 2) {
    if (port_generator_status_.read(genMsg) == RTT::NewData) {
      if(genMsg != trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::ACTIVE){
        //std::cout<<genMsg<<std::endl;
      }
      if(genMsg == trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL){
        //std::cout<<"got SUCCESSFUL"<<std::endl;
        if (goalToleranceIsViolated(g)) {
          //std::cout<<"GOAL_TOLERANCE_VIOLATED"<<std::endl;
          res.error_code =
              trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
          activeGoal_.setAborted(res, "");
          goal_active_ = false;
        } else {
          //std::cout<<"SUCCESSFUL"<<std::endl;
          res.error_code = trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL;
          activeGoal_.setSucceeded(res, "");
          goal_active_ = false;
        }
      } else if(genMsg == trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::ACTIVE){
        ros::Time now = rtt_rosclock::host_now();
        // fealing feedback msg
        //std::cout<<"sending feedback"<<std::endl;
        for (int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
          feedback_.actual.positions[i] = joint_position_[i];
          feedback_.desired.positions[i] = desired_joint_position_[i];
          feedback_.error.positions[i] = joint_position_[i]
              - desired_joint_position_[i];
        }
        feedback_.header.stamp = rtt_rosclock::host_now();
        activeGoal_.publishFeedback(feedback_);
      } else {
        res.error_code = genMsg;
        activeGoal_.setAborted(res, "");
        goal_active_ = false;
      }
    } else {
      //std::cout<<"no data from generator"<<std::endl;
      res.error_code = trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::INACTIVE;
      activeGoal_.setAborted(res, "");
      goal_active_ = false;
    }
  }
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::goalCB(GoalHandle gh) {
  //std::cout<<"deof: "<<TRAJECTORY_TYPE::DOFS<<std::endl;
  if (!goal_active_) {
    trajectory_msgs::JointTrajectory* trj_ptr =
        new trajectory_msgs::JointTrajectory;
    Goal g = gh.getGoal();

    trapezoid_trajectory_msgs::TrapezoidTrajectoryResult res;

    RTT::Logger::log(RTT::Logger::Debug) << "Received trajectory contain "
        << g->trajectory.points.size() << " points" << RTT::endlog();

    if (g->trajectory.points.size() > jnt_command_out_.trj.size()) {
        RTT::Logger::log(RTT::Logger::Error)
            << "Trajectory contains too many points" << RTT::endlog();
        res.error_code =
            trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::INVALID_JOINTS;
        gh.setRejected(res, "");
        goal_active_ = false;
        return;
    }

    if(!fillRemapTable(g)){
      res.error_code =
          trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(res, "");
      goal_active_ = false;
      return;
    }
    //std::cout<<"filled remap table"<<std::endl;
    // Sprawdzenie ograniczeń w jointach INVALID_GOAL
    if(trajectoryHasInvalidPoints(g)){
      res.error_code = trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::INVALID_GOAL;
      gh.setRejected(res, "");
      goal_active_ = false;
      return;
    }
    //std::cout<<"trajectory has no invalid points"<<std::endl;
    // Remap joints
    /*if(!remapJointsAndLimits(trj_ptr, g, max_vel, max_acc)){
      res.error_code = trapezoid_trajectory_msgs::
                        TrapezoidTrajectoryResult::
                        INVALID_LIMIT_ARRAY;
      gh.setRejected(res, "");
      goal_active_ = false;
      return;
    }*/
    if((g->max_velocities.size() != TRAJECTORY_TYPE::DOFS ||
       g->max_accelerations.size() != TRAJECTORY_TYPE::DOFS) &&
       !g->duration_mode){
      RTT::Logger::log(RTT::Logger::Debug) << 
            "not enough limit values for velocites for"<<
            " accelerations for research mode";
      res.error_code = trapezoid_trajectory_msgs::
                        TrapezoidTrajectoryResult::
                        INVALID_LIMIT_ARRAY;
      gh.setRejected(res, "");
      goal_active_ = false;
      return;
    }

    
    // Sprawdzenie czasu w nagłówku OLD_HEADER_TIMESTAMP
    if (g->trajectory.header.stamp < rtt_rosclock::host_now()) {
      RTT::Logger::log(RTT::Logger::Debug) << "Old header timestamp"
          << RTT::endlog();
      res.error_code =trapezoid_trajectory_msgs::
                        TrapezoidTrajectoryResult::OLD_HEADER_TIMESTAMP;
      gh.setRejected(res, "");
      goal_active_ = false;
    }

    activeGoal_ = gh;
    goal_active_ = true;

    if (getPeersReady()) {
      TRAJECTORY_TYPE trj_cptr = TRAJECTORY_TYPE();
      //std::cout<<"got peers ready"<<std::endl;
      trj_cptr.start = g->trajectory.header.stamp;
      trj_cptr.count_trj = g->trajectory.points.size();
      for (unsigned int i = 0; i < g->trajectory.points.size(); i++) {
        for (unsigned int j = 0; j < g->trajectory.points[i].positions.size(); j++) {
          trj_cptr.trj[i].positions[j] = g->trajectory.points[i].positions[remapTable_[j]];
        }

        for (unsigned int j = 0; j < g->trajectory.points[i].velocities.size();
            j++) {
          trj_cptr.trj[i].velocities[j] = g->trajectory.points[i].velocities[remapTable_[j]];
        }
      trj_cptr.trj[i].time_from_start = g->trajectory.points[i].time_from_start;
      }

      // prepare tolerances data
      if (g->path_tolerance.size() == g->trajectory.joint_names.size()) {
       for (int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
          trj_cptr.path_tolerance[i] = g->path_tolerance[remapTable_[i]].position;
        }
      }
      if (g->goal_tolerance.size() == g->trajectory.joint_names.size()) {
        for (int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
          trj_cptr.goal_tolerance[i] = g->goal_tolerance[remapTable_[i]].position;
        }
      }
      if (g->max_velocities.size() == g->trajectory.joint_names.size()) {
        for (int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
          trj_cptr.max_velocities[i] = g->max_velocities[remapTable_[i]];
        }
      }
      if (g->max_accelerations.size() == g->trajectory.joint_names.size()) {
        for (int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
          trj_cptr.max_accelerations[i] = g->max_accelerations[remapTable_[i]];
        }
      }
      trj_cptr.save_data = g->save_data;
      trj_cptr.research_mode = g->research_mode;
      trj_cptr.duration_mode = g->duration_mode;
      for (unsigned int j = 0; j < TRAJECTORY_TYPE::DOFS;j++) {
        trj_cptr.stiffness[j] = stiffness_(j);
      }
      port_jnt_command_out_.write(trj_cptr);
      ////std::cout<<"send data to generator"<<std::endl;
      gh.setAccepted();
      goal_active_ = true;
      cycles_ = 0;
    } else {
      gh.setRejected();
      goal_active_ = false;
    }
  } else {
    gh.setRejected();
  }
  //std::cout<<"finished goal pocessing"<<std::endl;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::cancelCB(GoalHandle gh) {
  goal_active_ = false;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::pathToleranceBreached(Goal g){

  if(g->trajectory.points.size()<1){
    return false;
  }
  if(g->research_mode){
    return false;
  }
  for (int i = 0; i < g->path_tolerance.size(); i++) {
    for (int j = 0; j < jointNames_.size(); j++) {
      if (jointNames_[j] == g->path_tolerance[i].name) {
        if (fabs(joint_position_[j] - desired_joint_position_[j])
            > g->path_tolerance[i].position) {
          /*std::cout<<"Path tolerance violated with oint_position_="
                  <<joint_position_[j]
                  <<" desired_joint_position_="
                  <<desired_joint_position_[j]
                  <<" fabs="
                  <<fabs(joint_position_[j] - desired_joint_position_[j])
                  <<"tolerance= "
                  <<g->path_tolerance[i]
                  <<std::endl;*/
          RTT::Logger::log(RTT::Logger::Error) << "Path tolerance violated"
              << RTT::endlog();
          return true;
        }
      }
    }
  }
  return false;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::fillRemapTable(Goal g){

    unsigned int j;
    for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
      for (j = 0; j < g->trajectory.joint_names.size(); j++) {
        if (g->trajectory.joint_names[j] == jointNames_[i]) {
          remapTable_[i] = j;
          break;
        }
      }
      if (j == g->trajectory.joint_names.size()) {
        RTT::Logger::log(RTT::Logger::Error)
            << "Trajectory contains invalid joint" << RTT::endlog();
        return false;
      }
    }
    return true;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::trajectoryHasInvalidPoints(Goal g){
  for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
    for (int j = 0; j < g->trajectory.points.size(); j++) {
      const double joint_position = g->trajectory.points[j].positions[i];
      if (joint_position > upperLimits_[remapTable_[i]] || 
          joint_position < lowerLimits_[remapTable_[i]]) 
      {
        RTT::Logger::log(RTT::Logger::Debug) << 
            "Invalid goal [" << i << "]: " << 
            upperLimits_[remapTable_[i]] << ">" << 
            joint_position << ">" << 
            lowerLimits_[remapTable_[i]] << RTT::endlog();
        return true;
      }
    }
  }
  return false;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::remapJointsAndLimits(
                trajectory_msgs::JointTrajectory* trj_ptr, Goal g, double* max_vel, double* max_acc){

  trj_ptr->header = g->trajectory.header;
  trj_ptr->points.resize(g->trajectory.points.size());


  if(!g->duration_mode){
    //check wether right ammount of vel and acc limitations was given
    if(g->max_velocities.size() != TRAJECTORY_TYPE::DOFS ||
       g->max_accelerations.size() != TRAJECTORY_TYPE::DOFS){
      RTT::Logger::log(RTT::Logger::Debug) << 
            "not enough limit values for velocites for"<<
            " accelerations for research mode";
      return false;
    } else {
      //remap max_vel and max_acc tables
      for (unsigned int j = 0; j < TRAJECTORY_TYPE::DOFS;j++) {
        max_vel[j] = g->max_velocities[remapTable_[j]];
        max_acc[j] = g->max_accelerations[remapTable_[j]];
      }
    }
  }
  for (unsigned int i = 0; i < g->trajectory.points.size(); i++) {
    trj_ptr->points[i].positions.resize(
        g->trajectory.points[i].positions.size());
    for (unsigned int j = 0; j < g->trajectory.points[i].positions.size();
        j++) {
      trj_ptr->points[i].positions[j] =
          g->trajectory.points[i].positions[remapTable_[j]];
    }

    trj_ptr->points[i].velocities.resize(
        g->trajectory.points[i].velocities.size());
    for (unsigned int j = 0; j < g->trajectory.points[i].velocities.size();
        j++) {
      trj_ptr->points[i].velocities[j] =
          g->trajectory.points[i].velocities[remapTable_[j]];
    }

    trj_ptr->points[i].accelerations.resize(
        g->trajectory.points[i].accelerations.size());
    for (unsigned int j = 0; j < g->trajectory.points[i].accelerations.size();
        j++) {
      trj_ptr->points[i].accelerations[j] = g->trajectory.points[i]
          .accelerations[remapTable_[j]];
    }

    trj_ptr->points[i].time_from_start = g->trajectory.points[i]
        .time_from_start;
  }

  return true;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::getPeersReady(){
  bool peers_ready_ = true;

  RTT::TaskContext::PeerList peers = this->getPeerList();
  for (size_t i = 0; i < peers.size(); i++) {
    RTT::Logger::log(RTT::Logger::Debug) << "Starting peer : " << peers[i]
        << RTT::endlog();
    peers_ready_ = peers_ready_ && this->getPeer(peers[i])->start();
  }
  return peers_ready_;
}


template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryAction<TRAJECTORY_TYPE>::goalToleranceIsViolated(Goal g)
{
  if(g->trajectory.points.size()<1){
    return false;
  }
  for (int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) { //iteration over joints
    for (int j = 0; j < g->goal_tolerance.size(); j++) { //interation over goal tolerance
      if (g->goal_tolerance[j].name == g->trajectory.joint_names[i]) { //checking wether in message tolerance's name matches name's position
        //checking wether finall position is near enough (+/- tolerance) of the given goal 
        if (joint_position_[remapTable_[i]] + g->goal_tolerance[j].position
              < g->trajectory.points[g->trajectory.points.size() - 1].positions[i]
            || joint_position_[remapTable_[i]]- g->goal_tolerance[j].position
              > g->trajectory.points[g->trajectory.points.size() - 1].positions[i]) 
        {
          RTT::Logger::log(RTT::Logger::Debug) <<"Goal tolerance "
                                               << g->goal_tolerance[j].name
                                               << " violated with position "
                                               << joint_position_[remapTable_[i]]
                                               << RTT::endlog();
          return true;
        }
      }
    }
  }
  return false;
}

/*bool InternalSpaceTrapezoidTrajectoryAction::check_trajectory_limits(JointTrajectoryPoint point, int remapTable_index){

}*/

#endif  // CONTROLLER_COMMON_INTERNAL_SPACE_SPLINE_TRAJECTORY_ACTION_H_