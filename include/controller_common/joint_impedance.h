// Copyright 2016 WUT
/*
 * joint_limit_avoidance.h
 *
 *  Created on: 30 sep 2014
 *      Author: konradb3, dseredyn
 */

#ifndef JOINT_IMPEDANCE_H_
#define JOINT_IMPEDANCE_H_

#include <string>
#include <vector>

#include "Eigen/Dense"

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

template <unsigned NUMBER_OF_JOINTS>
class JointImpedance: public RTT::TaskContext {
 public:
  explicit JointImpedance(const std::string& name);
  virtual ~JointImpedance();

  bool configureHook();
  bool startHook();
  void stopHook();
  void updateHook();

 protected:

  typedef Eigen::Matrix<double, NUMBER_OF_JOINTS, 1>  VectorNd;
  typedef Eigen::Matrix<double, NUMBER_OF_JOINTS, NUMBER_OF_JOINTS>  MatrixNd;

  RTT::InputPort<VectorNd> port_joint_position_;
  RTT::InputPort<VectorNd> port_joint_position_command_;
  RTT::InputPort<VectorNd> port_joint_stiffness_command_;
  RTT::InputPort<VectorNd> port_joint_velocity_;
  RTT::InputPort<MatrixNd> port_mass_matrix_;
  RTT::InputPort<VectorNd> port_nullspace_torque_command_;
  RTT::OutputPort<VectorNd> port_joint_torque_command_;

  VectorNd joint_position_;
  VectorNd joint_position_command_;
  VectorNd joint_error_;
  VectorNd joint_velocity_;
  VectorNd joint_torque_command_;
  VectorNd nullspace_torque_command_;

  std::vector<double> initial_stiffness_;
  double damping_;

  Eigen::GeneralizedSelfAdjointEigenSolver<MatrixNd > es_;

  VectorNd k_, k0_;
  MatrixNd m_, d_, q_;
  MatrixNd tmpNN_;
};

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

using namespace RTT;

template <unsigned NUMBER_OF_JOINTS>
JointImpedance<NUMBER_OF_JOINTS>::JointImpedance(const std::string& name)
    : RTT::TaskContext(name, PreOperational)
    , port_joint_torque_command_("JointTorqueCommand_OUTPORT", true)
    , damping_(0.7)
{

  this->ports()->addPort("JointPosition_INPORT", port_joint_position_);
  this->ports()->addPort("JointPositionCommand_INPORT", port_joint_position_command_);
  this->ports()->addPort("JointStiffnessCommand_INPORT", port_joint_stiffness_command_);
  this->ports()->addPort("JointVelocity_INPORT", port_joint_velocity_);
  this->ports()->addPort("MassMatrix_INPORT", port_mass_matrix_);
  this->ports()->addPort(port_joint_torque_command_);
  this->ports()->addPort("NullSpaceTorqueCommand_INPORT",
                         port_nullspace_torque_command_);

  this->properties()->addProperty("initial_stiffness", initial_stiffness_);
  this->properties()->addProperty("damping", damping_);
}

template <unsigned NUMBER_OF_JOINTS>
JointImpedance<NUMBER_OF_JOINTS>::~JointImpedance() {
}

template <unsigned NUMBER_OF_JOINTS>
bool JointImpedance<NUMBER_OF_JOINTS>::configureHook() {
  Logger::In in("JointImpedance::configureHook");

  if ((initial_stiffness_.size() != NUMBER_OF_JOINTS)) {
    log(RTT::Error) << "invalid size of initial_stiffness: "
        << initial_stiffness_.size() << ", should be "
        << NUMBER_OF_JOINTS << Logger::endl;
    return false;
  }

  nullspace_torque_command_.setZero();

  if (damping_ < 0.5 || damping_ > 2.0) {
    log(RTT::Error) << "wrong value of damping: " << damping_
        << ", should be in range [0.5, 2.0]"
        << Logger::endl;
    return false;
  }

  for (size_t i = 0; i < initial_stiffness_.size(); i++) {
    k_(i) = initial_stiffness_[i];
    if (k_(i) < 0.0 || k_(i) > 3000) {
      log(RTT::Error) << "wrong value of initial_stiffness[" << i
          << "]: " << k_(i) << ", should be in range [0, 3000]"
          << Logger::endl;
      return false;
    }
  }

  port_joint_torque_command_.setDataSample(joint_torque_command_);

  return true;
}

template <unsigned NUMBER_OF_JOINTS>
bool JointImpedance<NUMBER_OF_JOINTS>::startHook() {
  RESTRICT_ALLOC;
  return true;
}

template <unsigned NUMBER_OF_JOINTS>
void JointImpedance<NUMBER_OF_JOINTS>::stopHook() {
  for (int i = 0; i < joint_torque_command_.size(); i++) {
    joint_torque_command_(i) = 0.0;
  }

  port_joint_torque_command_.write(joint_torque_command_);

  UNRESTRICT_ALLOC;
}

template <unsigned NUMBER_OF_JOINTS>
void JointImpedance<NUMBER_OF_JOINTS>::updateHook() {

  if (port_joint_position_.read(joint_position_) != RTT::NewData) {
    error();
    Logger::In in("JointImpedance::updateHook");
    Logger::log() << Logger::Error << "no data on port "
                  << port_joint_position_.getName() << Logger::endl;
    return;
  }

  if (port_joint_position_command_.read(joint_position_command_) != RTT::NewData) {
    error();
    Logger::In in("JointImpedance::updateHook");
    Logger::log() << Logger::Error << "no data on port "
                  << port_joint_position_command_.getName() << Logger::endl;
    return;
  }

  if (port_joint_stiffness_command_.read(k_) != RTT::NewData) {
    std::cout<<k_<<std::endl;
    std::cout<<"No stiffness"<<std::endl;
    error();
    Logger::In in("JointImpedance::updateHook");
    Logger::log() << Logger::Error << "no data on port "
                  << port_joint_stiffness_command_.getName() << Logger::endl;
    return;
  }

  if (port_joint_velocity_.read(joint_velocity_) != RTT::NewData) {
    error();
    Logger::In in("JointImpedance::updateHook");
    Logger::log() << Logger::Error << "no data on port "
                  << port_joint_velocity_.getName() << Logger::endl;
    return;
  }

  // this port can be unconnected
  if (port_nullspace_torque_command_.read(nullspace_torque_command_) != RTT::NewData) {
    nullspace_torque_command_.setZero();
  }

  joint_error_.noalias() = joint_position_command_ - joint_position_;
  joint_torque_command_.noalias() = k_.cwiseProduct(joint_error_);

  if (port_mass_matrix_.read(m_) != RTT::NewData) {
    error();
    Logger::In in("JointImpedance::updateHook");
    Logger::log() << Logger::Error << "no data on port "
                  << port_mass_matrix_.getName() << Logger::endl;
    return;
  }

  tmpNN_ = k_.asDiagonal();

  es_.compute(tmpNN_, m_);

  q_ = es_.eigenvectors().inverse();

  k0_ = es_.eigenvalues();

  tmpNN_ = k0_.cwiseAbs().cwiseSqrt().asDiagonal();

//  d_.noalias() = 2.0 * q_.transpose() * 0.7 * tmpNN_ * q_;    // hard-coded damping is replaced by ROS param damping
  d_.noalias() = 2.0 * q_.transpose() * damping_ * tmpNN_ * q_;

  if (!joint_torque_command_.allFinite()) {
    RTT::Logger::In in("JointImpedance::updateHook");
    error();
    RTT::log(RTT::Error) << "Non finite output form stiffness" << Logger::endl;
  }

  joint_torque_command_.noalias() -= d_ * joint_velocity_;

  if (!joint_torque_command_.allFinite()) {
    RTT::Logger::In in("JointImpedance::updateHook");
    error();
    RTT::log(RTT::Error) << "Non finite output form damping" << Logger::endl;
  }

  joint_torque_command_.noalias() += nullspace_torque_command_;

  if (!joint_torque_command_.allFinite()) {
    RTT::Logger::In in("JointImpedance::updateHook");
    error();
    RTT::log(RTT::Error) << "Non finite output form nullspace" << Logger::endl;
  }

  port_joint_torque_command_.write(joint_torque_command_);
}

#endif  // JOINT_IMPEDANCE_H_
