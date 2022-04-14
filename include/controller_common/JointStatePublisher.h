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

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include "rtt_rosclock/rtt_rosclock.h"
#include <rtt_rosparam/rosparam.h>

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
  void addChildren(const KDL::SegmentMap::const_iterator segment);

  sensor_msgs::JointState joint_state_;
  Joints joint_position_;
  Joints joint_velocity_;
  Joints joint_effort_;
  std::vector<std::string> names_;
  std::vector<std::string> constant_names_;
  std::vector<double> constant_positions_;
  std::string robot_description_;
  std::vector<std::string > urdf_joint_names_fixed_;
  std::vector<std::string > urdf_joint_names_;
  urdf::Model model_;
};

template <unsigned DOFS>
JointStatePublisher<DOFS>::JointStatePublisher(const std::string& name)
    : RTT::TaskContext(name, PreOperational)
    , joint_names_prop("joint_names")
    , constant_names_prop("constant_names")
    , constant_positions_prop("constant_positions")
{
  ports()->addPort("JointPosition", port_joint_position_);
  ports()->addPort("JointVelocity", port_joint_velocity_);
  ports()->addPort("JointEffort", port_joint_effort_);
  ports()->addPort("joint_state", joint_state_port_);

  this->addProperty(joint_names_prop);
  this->addProperty(constant_names_prop);
  this->addProperty(constant_positions_prop);
  this->addProperty("robot_description", robot_description_);
}

template <unsigned DOFS>
JointStatePublisher<DOFS>::~JointStatePublisher() {
}

// add children to correct maps
template <unsigned DOFS>
void JointStatePublisher<DOFS>::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
  for (size_t i = 0; i < children.size(); ++i) {
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model_.getJoint(child.getJoint().getName()) &&
                    model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
        Logger::log() << Logger::Info << "Floating joint. Not adding segment from " << root
                    << " to " << child.getName()
                    << ". This TF can not be published based on joint_states info" << Logger::endl;
      }
      else {
        urdf_joint_names_fixed_.push_back(child.getJoint().getName());
        Logger::log() << Logger::Debug << "Adding fixed segment from " << root
                    << " to " << child.getName() << Logger::endl;
      }
    }
    else {
      urdf_joint_names_.push_back(child.getJoint().getName());
      Logger::log() << Logger::Debug << "Adding moving segment from " << root
                    << " to " << child.getName() << Logger::endl;
    }
    addChildren(children[i]);
  }
}

template <unsigned DOFS>
bool JointStatePublisher<DOFS>::configureHook() {
  RTT::Logger::In in("JointStatePublisher::configureHook");

  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
          this->getProvider<rtt_rosparam::ROSParam>("rosparam");

  if (!rosparam) {
    Logger::log() << Logger::Error << "Could not run rtt ROSParam" << Logger::endl;
    return false;
  }
  // Get the ROS parameter "/robot_description"
  
  if (!rosparam->getAbsolute("robot_description")) {
    Logger::log() << Logger::Error << "Could not get rtt ROSParam 'robot_description'"
                                                                              << Logger::endl;
    return false;
  }

  // Read robot_rescription, create URDF Model and create KDL tree
  if (robot_description_.empty()) {
    Logger::log() << Logger::Error << "ROS param robot_description is empty"
                         << Logger::endl;
    return false;
  }

  if (!model_.initString(robot_description_)) {
    Logger::log() << Logger::Error << "Could not initialize URDF Model for robot_description"
                                                                                  << Logger::endl;
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model_, tree)) {
    Logger::log() << Logger::Error <<"Failed to extract kdl tree from xml robot description"
                                                                                  << Logger::endl;
    return false;
  }

  addChildren(tree.getRootSegment());

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

  std::vector<std::string > names_verified;
  for (int i = 0; i < names_.size(); ++i) {
    bool found = false;
    for (int j = 0; j < urdf_joint_names_.size(); ++j) {
      if (urdf_joint_names_[j] == names_[i]) {
        found = true;
      }
    }
    if (found) {
      names_verified.push_back( names_[i] );
    }
  }
  names_ = names_verified;

  std::vector<std::string > constant_names_verified;
  std::vector<double > constant_positions_verified;
  for (int i = 0; i < constant_names_.size(); ++i) {
    bool found = false;
    for (int j = 0; j < urdf_joint_names_fixed_.size(); ++j) {
      if (urdf_joint_names_fixed_[j] == constant_names_[i]) {
        found = true;
      }
    }
    for (int j = 0; j < urdf_joint_names_.size(); ++j) {
      if (urdf_joint_names_[j] == constant_names_[i]) {
        found = true;
      }
    }
    if (found) {
      constant_names_verified.push_back( constant_names_[i] );
      constant_positions_verified.push_back( constant_positions_[i] );
    }
  }
  constant_names_ = constant_names_verified;
  constant_positions_ = constant_positions_verified;

  Logger::log() << Logger::Info << "moveable joints:" << Logger::endl;
  std::cout << "moveable joints:" << std::endl;
  for (int i = 0; i < names_.size(); ++i) {
    Logger::log() << Logger::Info << "  " << names_[i] << Logger::endl;
    std::cout << "  " << names_[i] << std::endl;
  }

  Logger::log() << Logger::Info << "fixed joints:" << Logger::endl;
  std::cout << "fixed joints:" << std::endl;
  for (int i = 0; i < constant_names_.size(); ++i) {
    Logger::log() << Logger::Info << "  " << constant_names_[i] << Logger::endl;
    std::cout << "  " << constant_names_[i] << std::endl;
  }

  int DOFS_verified = names_.size() + constant_names_.size();
  joint_state_.name.resize( DOFS_verified );
  joint_state_.position.resize( DOFS_verified );
  joint_state_.velocity.resize( DOFS_verified );
  joint_state_.effort.resize( DOFS_verified );

  for (unsigned int i = 0; i < names_.size(); i++) {
    joint_state_.name[i] = names_[i].c_str();
  }

  for (unsigned int i = 0; i < constant_names_.size(); i++) {
    joint_state_.name[names_.size()+i] = constant_names_[i];
  }

  return true;
}

template <unsigned DOFS>
void JointStatePublisher<DOFS>::updateHook() {
  if (port_joint_position_.read(joint_position_) == RTT::NewData) {
    port_joint_velocity_.read(joint_velocity_);
    port_joint_effort_.read(joint_effort_);
    joint_state_.header.stamp = rtt_rosclock::host_now();
    for (unsigned int i = 0; i < names_.size(); i++) {
      joint_state_.position[i] = joint_position_[i];
      joint_state_.velocity[i] = joint_velocity_[i];
      joint_state_.effort[i] = joint_effort_[i];
    }
    for (unsigned int i = 0; i < constant_positions_.size(); i++) {
      joint_state_.position[names_.size() + i] = constant_positions_[i];
      joint_state_.velocity[names_.size() + i] = 0;
      joint_state_.effort[names_.size() + i] = 0;
    }
    joint_state_port_.write(joint_state_);
  }
}

#endif  // JOINTSTATEPUBLISHER_H__
