// Copyright 2014 WUT
/*
 * robot_mass_matrix.h
 *
 *  Created on: 12 mar 2014
 *      Author: konradb3
 */

#ifndef ROBOT_MASS_MATRIX_H_
#define ROBOT_MASS_MATRIX_H_

#include <string>

#include "Eigen/Dense"

#include "rtt/TaskContext.hpp"

#include "controller_common/robot.h"

using namespace RTT;

template <unsigned DOFS, unsigned EFFECTORS>
class RobotMassMatrix: public RTT::TaskContext {
 public:
  explicit RobotMassMatrix(const std::string& name);
  virtual ~RobotMassMatrix();

  bool configureHook();
  void updateHook();
 private:
  typedef Eigen::Matrix<double, DOFS, 1> Joints;
  typedef Eigen::Matrix<double, DOFS, DOFS> Inertia;

  RTT::InputPort<Joints> port_joint_position_;
  RTT::OutputPort<Inertia> port_mass_matrix_;
  RTT::OutputPort<Inertia> port_mass_matrix_inv_;

  boost::shared_ptr<controller_common::Robot<DOFS, EFFECTORS> > robot_;

  Joints joint_position_;
  Inertia M_, Mi_;
  Eigen::PartialPivLU<Inertia> lu_;
};

template <unsigned DOFS, unsigned EFFECTORS>
RobotMassMatrix<DOFS, EFFECTORS>::RobotMassMatrix(const std::string& name) : RTT::TaskContext(name, PreOperational) {

  this->ports()->addPort("JointPosition", port_joint_position_);
  this->ports()->addPort("MassMatrix", port_mass_matrix_);
  this->ports()->addPort("MassMatrixInv", port_mass_matrix_inv_);
}

template <unsigned DOFS, unsigned EFFECTORS>
RobotMassMatrix<DOFS, EFFECTORS>::~RobotMassMatrix() {
}

template <unsigned DOFS, unsigned EFFECTORS>
bool RobotMassMatrix<DOFS, EFFECTORS>::configureHook() {
  robot_ = this->getProvider<controller_common::Robot<DOFS, EFFECTORS> >("robot");
  if (!robot_) {
    Logger::log() << Logger::Error << "Unable to load RobotService"
                         << Logger::endl;
    return false;
  }

  port_mass_matrix_.setDataSample(M_);
  port_mass_matrix_inv_.setDataSample(Mi_);

  return true;
}

template <unsigned DOFS, unsigned EFFECTORS>
void RobotMassMatrix<DOFS, EFFECTORS>::updateHook() {
  port_joint_position_.read(joint_position_);

  if (joint_position_.size() != DOFS) {
    Logger::log() << Logger::Error << "Received joint position vector have invalid size. [read: "
                         << joint_position_.size() << " expected: " << DOFS
                         << "]" << Logger::endl;
    error();
    return;
  }

  robot_->inertia(M_, joint_position_, 0);

  lu_.compute(M_);
  Mi_ = lu_.inverse();

  port_mass_matrix_.write(M_);
  port_mass_matrix_inv_.write(Mi_);
}

#endif  // ROBOT_MASS_MATRIX_H_
