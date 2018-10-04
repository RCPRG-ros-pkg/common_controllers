// Copyright 2014 WUT

/*
 * robot_mass_matrix.h
 *
 *  Created on: 12 mar 2014
 *      Author: konradb3
 */

#ifndef MASS_TEST_H_
#define MASS_TEST_H_

#include <string>

#include "Eigen/Dense"

#include "rtt/TaskContext.hpp"

#include "controller_common/robot.h"

using namespace RTT;

template <unsigned DOFS, unsigned EFFECTORS>
class MassTest: public RTT::TaskContext {
 public:
  explicit MassTest(const std::string& name);
  virtual ~MassTest();

  bool configureHook();
  void updateHook();
 private:
  typedef Eigen::Matrix<double, DOFS, 1> Joints;
  typedef Eigen::Matrix<double, DOFS, DOFS> Inertia;

  RTT::InputPort<Joints > port_joint_position_;
  RTT::OutputPort<Inertia> port_mass_matrix_;
  RTT::InputPort<Eigen::Matrix<double, 7, 7> > port_mass_matrix_left_;
  RTT::InputPort<Eigen::Matrix<double, 7, 7> > port_mass_matrix_right_;

  boost::shared_ptr<controller_common::Robot<DOFS, EFFECTORS> > robot_;

  Inertia M_;
  Eigen::Matrix<double, 7, 7> Ml_, Mr_;
  Joints joint_position_;
};

template <unsigned DOFS, unsigned EFFECTORS>
MassTest<DOFS, EFFECTORS>::MassTest(const std::string& name) :
    RTT::TaskContext(name, PreOperational),
    port_joint_position_("JointPosition_INPORT"),
    port_mass_matrix_("MassMatrix_OUTPORT", false),
    port_mass_matrix_left_("MassMatrixLeft_INPORT"),
    port_mass_matrix_right_("MassMatrixRight_INPORT") {

  this->ports()->addPort(port_joint_position_);
  this->ports()->addPort(port_mass_matrix_);
  this->ports()->addPort(port_mass_matrix_left_);
  this->ports()->addPort(port_mass_matrix_right_);
}

template <unsigned DOFS, unsigned EFFECTORS>
MassTest<DOFS, EFFECTORS>::~MassTest() {
}

template <unsigned DOFS, unsigned EFFECTORS>
bool MassTest<DOFS, EFFECTORS>::configureHook() {
  RTT::Logger::In in("MassTest::configureHook");

  robot_ = this->getProvider<controller_common::Robot<DOFS, EFFECTORS> >("robot");
  if (!robot_) {
    Logger::log() << Logger::Error << "Unable to load RobotService"
                         << Logger::endl;
    return false;
  }

  if (robot_->dofs() != DOFS) {
    Logger::log() << Logger::Error << "wrong number of DOFs: " << robot_->dofs()
                         << ", expected " << DOFS << Logger::endl;
    return false;
  }

  if (robot_->effectors() != EFFECTORS) {
    Logger::log() << Logger::Error << "wrong number of effectors: " << robot_->effectors()
                         << ", expected " << EFFECTORS << Logger::endl;
    return false;
  }

  port_mass_matrix_.setDataSample(M_);

  return true;
}

template <unsigned DOFS, unsigned EFFECTORS>
void MassTest<DOFS, EFFECTORS>::updateHook() {
  port_joint_position_.read(joint_position_);

  if (joint_position_.size() != DOFS) {
    RTT::Logger::In in("MassTest::updateHook");
    Logger::log() << Logger::Error << "Received joint position vector have invalid size. [read: "
                         << joint_position_.size() << " expected: " << DOFS
                         << "]" << Logger::endl;
    error();
    return;
  }

  //robot_->inertia(M_, joint_position_, 0);

  M_.setZero();

  M_(0, 0) = 10.0;

  if (port_mass_matrix_left_.read(Ml_) != RTT::NewData) {
    RTT::Logger::In in("MassTest::updateHook");
    Logger::log() << Logger::Error << "could not receive data on port "
                         << port_mass_matrix_left_.getName()
                         << Logger::endl;
    error();
    return;
  }
  if (Ml_.rows() != 7 || Ml_.cols() != 7) {
    RTT::Logger::In in("MassTest::updateHook");
    Logger::log() << Logger::Error << "invalid size of data received on port "
                         << port_mass_matrix_left_.getName()
                         << ": " << Ml_.rows() << " " << Ml_.cols()
                         << Logger::endl;
    error();
    return;
  }

  if (port_mass_matrix_right_.read(Mr_) != RTT::NewData) {
    RTT::Logger::In in("MassTest::updateHook");
    Logger::log() << Logger::Error << "could not receive data on port "
                         << port_mass_matrix_left_.getName()
                         << Logger::endl;
    error();
    return;
  }

  if (Mr_.rows() != 7 || Mr_.cols() != 7) {
    RTT::Logger::In in("MassTest::updateHook");
    Logger::log() << Logger::Error << "invalid size of data received on port "
                         << port_mass_matrix_right_.getName()
                         << ": " << Mr_.rows() << " " << Mr_.cols()
                         << Logger::endl;
    error();
    return;
  }

  M_.template block<7, 7>(1, 1) = Mr_;
  M_.template block<7, 7>(8, 8) = Ml_;

  port_mass_matrix_.write(M_);
}

#endif  // MASS_TEST_H_
