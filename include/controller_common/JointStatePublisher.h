/*
 * Copyright (c) 2010-2015 Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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

#ifndef JOINTSTATEPUBLISHER_H__
#define JOINTSTATEPUBLISHER_H__

#include <string>
#include <vector>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include "rtt_rosclock/rtt_rosclock.h"

#include <Eigen/Dense>

#include "sensor_msgs/JointState.h"

using namespace RTT;

template <unsigned DOFS>
class JointStatePublisher : public RTT::TaskContext
{
public:
  JointStatePublisher(const std::string& name);
  ~JointStatePublisher();

  bool configureHook();
  void updateHook();
protected:

  typedef Eigen::Matrix<double, DOFS, 1> Joints;

  RTT::InputPort<Joints > port_joint_position_;
  RTT::InputPort<Joints > port_joint_velocity_;
  RTT::InputPort<Joints > port_joint_effort_;
  RTT::OutputPort<sensor_msgs::JointState> joint_state_port_;

  RTT::Property<std::vector<std::string> > joint_names_prop;
  RTT::Property<std::vector<std::string> > constant_names_prop;
  RTT::Property<std::vector<double> > constant_positions_prop;
private:
  sensor_msgs::JointState joint_state_;
  Joints joint_position_;
  Joints joint_velocity_;
  Joints joint_effort_;
  std::vector<std::string> names_;
  std::vector<std::string> constant_names_;
  std::vector<double> constant_positions_;
};

template <unsigned DOFS>
JointStatePublisher<DOFS>::JointStatePublisher(const std::string& name) :
  RTT::TaskContext(name, PreOperational), joint_names_prop("joint_names",
      "number of joints"), constant_names_prop("constant_names"), constant_positions_prop("constant_positions") {
  ports()->addPort("JointPosition", port_joint_position_);
  ports()->addPort("JointVelocity", port_joint_velocity_);
  ports()->addPort("JointEffort", port_joint_effort_);
  ports()->addPort("joint_state", joint_state_port_);

  this->addProperty(joint_names_prop);
  this->addProperty(constant_names_prop);
  this->addProperty(constant_positions_prop);
}

template <unsigned DOFS>
JointStatePublisher<DOFS>::~JointStatePublisher() {
}

template <unsigned DOFS>
bool JointStatePublisher<DOFS>::configureHook() {
  RTT::Logger::In in("JointStatePublisher::configureHook");

  names_ = joint_names_prop.get();
  if (names_.empty()) {
    Logger::log() << Logger::Error << "ROS param joint_names is empty"
                         << Logger::endl;
    return false;
  }

  if (DOFS != names_.size()) {
    Logger::log() << Logger::Error << "ROS param joint_names has wrong size:"
                         << names_.size() << ", expected: "
                         << DOFS << Logger::endl;
    return false;
  }

  constant_names_ = constant_names_prop.get();
  constant_positions_ = constant_positions_prop.get();
  if (constant_names_.size() != constant_positions_.size()) {
    Logger::log() << Logger::Error << "ROS param constant_names should have the same size as constant_positions"
                         << constant_names_.size() << "!=" << constant_positions_.size()
                         << Logger::endl;
    return false;
  }

  joint_state_.name.resize(DOFS+constant_positions_.size());
  joint_state_.position.resize(DOFS+constant_positions_.size());
  joint_state_.velocity.resize(DOFS+constant_positions_.size());
  joint_state_.effort.resize(DOFS+constant_positions_.size());

  for (unsigned int i = 0; i < DOFS; i++) {
    joint_state_.name[i] = names_[i].c_str();
  }

  for (unsigned int i = 0; i < constant_names_.size(); i++) {
    joint_state_.name[DOFS+i] = constant_names_[i];
  }

  return true;
}

template <unsigned DOFS>
void JointStatePublisher<DOFS>::updateHook() {
  if (port_joint_position_.read(joint_position_) == RTT::NewData) {
    port_joint_velocity_.read(joint_velocity_);
    port_joint_effort_.read(joint_effort_);
    joint_state_.header.stamp = rtt_rosclock::host_now();
    for (unsigned int i = 0; i < DOFS; i++) {
      joint_state_.position[i] = joint_position_[i];
      joint_state_.velocity[i] = joint_velocity_[i];
      joint_state_.effort[i] = joint_effort_[i];
    }
    for (unsigned int i = 0; i < constant_positions_.size(); i++) {
      joint_state_.position[DOFS + i] = constant_positions_[i];
      joint_state_.velocity[DOFS + i] = 0;
      joint_state_.effort[DOFS + i] = 0;
    }
    joint_state_port_.write(joint_state_);
  }
}

#endif  // JOINTSTATEPUBLISHER_H__
