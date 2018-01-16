/*
 Copyright (c) 2014-2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CONTROLLER_COMMON_CART_IMP_ACTION_H_
#define CONTROLLER_COMMON_CART_IMP_ACTION_H_

#include <string>

#include "Eigen/Dense"

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include <rtt/Logger.hpp>

#include "cartesian_trajectory_msgs/CartImpAction.h"
#include "cartesian_trajectory_msgs/CartImpGoal.h"
#include "controller_common/cartesian_status.h"

#include <actionlib/server/action_server.h>
#include "rtt_actionlib/rtt_actionlib.h"
#include "rtt_actionlib/rtt_action_server.h"

#include "rtt_rosclock/rtt_rosclock.h"
#include "eigen_conversions/eigen_msg.h"

template <class TRAJECTORY_TYPE >
class CartImpAction: public RTT::TaskContext {
 private:
  typedef actionlib::ServerGoalHandle<cartesian_trajectory_msgs::CartImpAction> GoalHandle;
  typedef boost::shared_ptr<const cartesian_trajectory_msgs::CartImpGoal> Goal;

 public:
  explicit CartImpAction(const std::string& name);
  virtual ~CartImpAction();

  bool startHook();
  void updateHook();

    bool testTrj();

 private:
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);

  RTT::InputPort<int32_t> port_generator_status_;

  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_;
  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_command_;
  RTT::InputPort<geometry_msgs::Wrench> port_cartesian_wrench_;

  typename TRAJECTORY_TYPE::_pose_type pose_command_out_;
  RTT::OutputPort<typename TRAJECTORY_TYPE::_pose_type > port_pose_command_out_;

  typename TRAJECTORY_TYPE::_tool_type tool_command_out_;
  RTT::OutputPort<typename TRAJECTORY_TYPE::_tool_type > port_tool_command_out_;

  typename TRAJECTORY_TYPE::_imp_type imp_command_out_;
  RTT::OutputPort<typename TRAJECTORY_TYPE::_imp_type > port_imp_command_out_;

  rtt_actionlib::RTTActionServer<cartesian_trajectory_msgs::CartImpAction> as_;
  GoalHandle activeGoal_;

  int test_counter_;
  bool goal_active_;
  int cycles_;
};

template <class TRAJECTORY_TYPE >
CartImpAction<TRAJECTORY_TYPE >::CartImpAction(const std::string& name) 
    : RTT::TaskContext(name)
    , port_pose_command_out_("cart_pose_command_OUTPORT")
    , port_tool_command_out_("cart_tool_command_OUTPORT")
    , port_imp_command_out_("cart_imp_command_OUTPORT")
    , port_generator_status_("generator_status_INPORT")
    , port_cartesian_position_("CartesianPosition_INPORT")
    , port_cartesian_position_command_("CartesianPositionCommand_INPORT")
    , port_cartesian_wrench_("CartesianWrench_INPORT")
{
  as_.addPorts(this->provides());

  this->ports()->addPort(port_pose_command_out_);
  this->ports()->addPort(port_tool_command_out_);
  this->ports()->addPort(port_imp_command_out_);

  this->ports()->addPort(port_generator_status_);
  this->ports()->addPort(port_cartesian_position_);
  this->ports()->addPort(port_cartesian_position_command_);
  this->ports()->addPort(port_cartesian_wrench_);

  as_.registerGoalCallback(boost::bind(&CartImpAction::goalCB, this, _1));
  as_.registerCancelCallback(boost::bind(&CartImpAction::cancelCB, this, _1));
}

template <class TRAJECTORY_TYPE >
CartImpAction<TRAJECTORY_TYPE >::~CartImpAction() {
}

template <class TRAJECTORY_TYPE >
bool CartImpAction<TRAJECTORY_TYPE >::startHook() {
  if (as_.ready()) {
    as_.start();
  } else {
    return false;
  }
  test_counter_ = 0;

  goal_active_ = false;
  cycles_ = 0;

  return true;
}

template <class TRAJECTORY_TYPE >
void CartImpAction<TRAJECTORY_TYPE >::updateHook() {
  geometry_msgs::Pose pose_msr_in;
  port_cartesian_position_.read(pose_msr_in);

  geometry_msgs::Pose pose_cmd_in;
  port_cartesian_position_command_.read(pose_cmd_in);


  cartesian_trajectory_msgs::CartImpResult res;

  int32_t generator_status;
  if (port_generator_status_.read(generator_status) != RTT::NewData) {
    generator_status = 3;
  }

  if (cycles_ < 100) {
    ++cycles_;
  }

  if (goal_active_ && cycles_ > 2) {
    if (generator_status == 3) {
      res.error_code = cartesian_trajectory_msgs::CartImpResult::UNKNOWN_ERROR;
      activeGoal_.setAborted(res);
      goal_active_ = false;
    }
    else if (generator_status == cartesian_status::INACTIVE) {
      // do nothing
      res.error_code = cartesian_trajectory_msgs::CartImpResult::UNKNOWN_ERROR;
      activeGoal_.setAborted(res);
      goal_active_ = false;
    }
    else if (generator_status == cartesian_status::ACTIVE) {

      cartesian_trajectory_msgs::CartImpFeedback feedback;
      Eigen::Affine3d actual, desired, error;
      ros::Time now = rtt_rosclock::host_now();

      feedback.pose_desired = pose_cmd_in;
      feedback.pose_actual = pose_msr_in;

      tf::poseMsgToEigen(feedback.pose_actual, actual);
      tf::poseMsgToEigen(feedback.pose_desired, desired);
      error = actual.inverse() * desired;
      tf::poseEigenToMsg(error, feedback.pose_error);
      feedback.header.stamp = now;
      activeGoal_.publishFeedback(feedback);
    }
    else if (generator_status == cartesian_status::SUCCESSFUL) {
      res.error_code = cartesian_trajectory_msgs::CartImpResult::SUCCESSFUL;
      activeGoal_.setSucceeded(res, "");
      goal_active_ = false;
    }
    else if (generator_status == cartesian_status::PATH_TOLERANCE_VIOLATED) {
      res.error_code = cartesian_trajectory_msgs::CartImpResult::PATH_TOLERANCE_VIOLATED;
      activeGoal_.setAborted(res);
      goal_active_ = false;
    }
    else if (generator_status == cartesian_status::GOAL_TOLERANCE_VIOLATED) {
      res.error_code = cartesian_trajectory_msgs::CartImpResult::GOAL_TOLERANCE_VIOLATED;
      activeGoal_.setAborted(res, "");
      goal_active_ = false;
    }
  }
}

template <class TRAJECTORY_TYPE >
void CartImpAction<TRAJECTORY_TYPE >::goalCB(GoalHandle gh) {
    RTT::Logger::In in(std::string("CartImpAction(") + getName() + ")::goalCB");

    // cancel active goal
    if (activeGoal_.isValid() && (activeGoal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
        activeGoal_.setCanceled();
    }

    Goal g = gh.getGoal();

    if (g->pose_trj.points.size() > pose_command_out_.trj.size()) {
        cartesian_trajectory_msgs::CartImpResult res;
// TODO:
//        res.error_code = 
        activeGoal_.setRejected(res);
        RTT::Logger::log() << RTT::Logger::Info << "trajectory rejected: pose_trj size is too big" << RTT::Logger::endl;
        return;
    }

    if (g->tool_trj.points.size() > tool_command_out_.trj.size()) {
        cartesian_trajectory_msgs::CartImpResult res;
// TODO:
//        res.error_code = 
        activeGoal_.setRejected(res);
        RTT::Logger::log() << RTT::Logger::Info << "trajectory rejected: tool_trj size is too big" << RTT::Logger::endl;
        return;
    }

    if (g->imp_trj.points.size() > imp_command_out_.trj.size()) {
        cartesian_trajectory_msgs::CartImpResult res;
// TODO:
//        res.error_code = 
        activeGoal_.setRejected(res);
        RTT::Logger::log() << RTT::Logger::Info << "trajectory rejected: imp_trj size is too big" << RTT::Logger::endl;
        return;
    }

/*    if (g->pose_trj.points.size() == 0
            && g->imp_trj.points.size() == 0
            && g->tool_trj.points.size() == 0) {
        cartesian_trajectory_msgs::CartImpResult res;
// TODO:
//        res.error_code = 
        activeGoal_.setRejected(res);
        RTT::Logger::log() << RTT::Logger::Info << "trajectory rejected: size is zero" << RTT::Logger::endl;
        return;
    }
*/
    pose_command_out_.path_tolerance = g->path_tolerance;
    pose_command_out_.goal_tolerance = g->goal_tolerance;

    if (g->pose_trj.points.size() == 0 && g->tool_trj.points.size() == 0 &&
        g->imp_trj.points.size() == 0)
    {
        pose_command_out_.start = g->pose_trj.header.stamp;
        pose_command_out_.count = 0;
        port_pose_command_out_.write(pose_command_out_);
    }
    else {
        if (g->pose_trj.points.size() > 0) {
            pose_command_out_.start = g->pose_trj.header.stamp;
            for (int i = 0; i < g->pose_trj.points.size(); ++i) {
                pose_command_out_.trj[i] = g->pose_trj.points[i];
            }
            pose_command_out_.count = g->pose_trj.points.size();
            port_pose_command_out_.write(pose_command_out_);
        }

        if (g->tool_trj.points.size() > 0) {
            tool_command_out_.start = g->tool_trj.header.stamp;
            for (int i = 0; i < g->tool_trj.points.size(); ++i) {
                tool_command_out_.trj[i] = g->tool_trj.points[i];
            }
            tool_command_out_.count = g->tool_trj.points.size();
            port_tool_command_out_.write(tool_command_out_);
        }

        if (g->imp_trj.points.size() > 0) {
            imp_command_out_.start = g->imp_trj.header.stamp;
            for (int i = 0; i < g->imp_trj.points.size(); ++i) {
                imp_command_out_.trj[i] = g->imp_trj.points[i];
            }
            imp_command_out_.count = g->imp_trj.points.size();
            port_imp_command_out_.write(imp_command_out_);
        }
    }
    RTT::Logger::log() << RTT::Logger::Info << "sending trajectory..." << RTT::Logger::endl;

    gh.setAccepted();
    activeGoal_ = gh;

    goal_active_ = true;
    cycles_ = 0;
    test_counter_ = 0;
}

template <class TRAJECTORY_TYPE >
void CartImpAction<TRAJECTORY_TYPE >::cancelCB(GoalHandle gh) {
    if (activeGoal_ == gh) {
        pose_command_out_ = typename TRAJECTORY_TYPE::_pose_type();
        imp_command_out_ = typename TRAJECTORY_TYPE::_imp_type();
        tool_command_out_ = typename TRAJECTORY_TYPE::_tool_type();

        port_pose_command_out_.write(pose_command_out_);
        port_imp_command_out_.write(imp_command_out_);
        port_tool_command_out_.write(tool_command_out_);

        activeGoal_.setCanceled();
        goal_active_ = false;
    }
}

#endif  // CONTROLLER_COMMON_CART_IMP_ACTION_H_

