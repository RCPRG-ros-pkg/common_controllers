// Copyright 2014 WUT
/*
 * joint_limit_avoidance.h
 *
 *  Created on: 11 mar 2014
 *      Author: konradb3
 */

#ifndef JOINT_LIMIT_AVOIDANCE_H_
#define JOINT_LIMIT_AVOIDANCE_H_

#include <string>
#include <vector>

#include "Eigen/Dense"

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

using namespace RTT;

template <unsigned DOFS>
class JointLimitAvoidance: public RTT::TaskContext {
 public:
  explicit JointLimitAvoidance(const std::string& name);
  virtual ~JointLimitAvoidance();

  bool configureHook();
  void updateHook();
  bool startHook();
  void stopHook();

 private:
  double jointLimitTrq(double hl, double ll, double ls, double r_max, double q);

  typedef Eigen::Matrix<double, DOFS, DOFS> Inertia;
  typedef Eigen::Matrix<double, DOFS, 1> Stiffness;
  typedef Eigen::Matrix<double, DOFS, 1> Joints;

  RTT::InputPort<Joints> port_joint_position_;
  RTT::InputPort<Joints> port_joint_velocity_;
  RTT::InputPort<Inertia> port_mass_matrix_;
  RTT::InputPort<Joints> port_nullspace_torque_command_;
  RTT::OutputPort<Joints> port_joint_torque_command_;

  Joints joint_position_, joint_velocity_, joint_torque_command_, nullspace_torque_command_;

  boost::array<std::vector<double >, DOFS > limits_;

  std::vector<double> max_trq_, limit_range_;

  Eigen::GeneralizedSelfAdjointEigenSolver<Inertia > es_;
  Stiffness k_, k0_;  // local stiffness
  Inertia m_, d_, q_, qt_;
  Inertia tmpNN_;
};

template <unsigned DOFS>
JointLimitAvoidance<DOFS>::JointLimitAvoidance(const std::string& name) :
  RTT::TaskContext(name, PreOperational)
{
  this->ports()->addPort("JointPosition_INPORT", port_joint_position_);
  this->ports()->addPort("JointVelocity_INPORT", port_joint_velocity_);
  this->ports()->addPort("MassMatrix_INPORT", port_mass_matrix_);
  this->ports()->addPort("JointTorqueCommand_OUTPORT", port_joint_torque_command_);
  this->ports()->addPort("NullSpaceTorqueCommand_INPORT",
                         port_nullspace_torque_command_);

  for (int i = 0; i < DOFS; ++i) {
    std::stringstream ss;
    ss << "limits_" << i;
    this->properties()->addProperty(ss.str(), limits_[i]);
  }

  this->properties()->addProperty("limit_range", limit_range_);
  this->properties()->addProperty("max_trq", max_trq_);
}

template <unsigned DOFS>
JointLimitAvoidance<DOFS>::~JointLimitAvoidance() {
}

template <unsigned DOFS>
bool JointLimitAvoidance<DOFS>::configureHook() {
    RTT::Logger::In in("JointLimitAvoidance::configureHook");

    if ((limit_range_.size() != DOFS)
        || (max_trq_.size() != DOFS)) {
        Logger::log() << Logger::Error << "invalid configuration data size" << Logger::endl;
        return false;
    }

    for (int i = 0; i < DOFS; ++i) {
        if (limits_[i].size() < 2 || (limits_[i].size()%2) != 0) {
            Logger::log() << Logger::Error << "wrong limits for joint " << i << Logger::endl;
            return false;
        }
    }

    return true;
}

template <unsigned DOFS>
void JointLimitAvoidance<DOFS>::updateHook() {
  if (port_joint_position_.read(joint_position_) != RTT::NewData) {
    RTT::Logger::In in("JointLimitAvoidance::updateHook");
    error();
    Logger::log() << Logger::Error << "could not read port: " << port_joint_position_.getName() << Logger::endl;
    return;
  }

  if (port_joint_velocity_.read(joint_velocity_) != RTT::NewData) {
    RTT::Logger::In in("JointLimitAvoidance::updateHook");
    error();
    Logger::log() << Logger::Error << "could not read port: " << port_joint_velocity_.getName() << Logger::endl;
    return;
  }

  if (port_nullspace_torque_command_.read(nullspace_torque_command_) != RTT::NewData) {
    // this is acceptable
    nullspace_torque_command_.setZero();
  }

  for (size_t i = 0; i < DOFS; i++) {
    bool limit_ok = false;
    double lo_limit, up_limit;
    for (int j = 0; j < limits_[i].size(); j += 2) {
        if (limits_[i][j] < joint_position_(i) && joint_position_(i) < limits_[i][j+1]) {
            limit_ok = true;
            lo_limit = limits_[i][j];
            up_limit = limits_[i][j+1];
            break;
        }
    }
    if (!limit_ok) {
      // This may sometimes happen, but it is not a big problem
      // The position in the joint violates the limit, so we have to search the closest violated
      // limit and apply the repulsive force anyway
      int closest_limit_idx = -1;
      double closest_limit_dist = 10000000.0;
      for (int j = 0; j < limits_[i].size(); j += 2) {
          double dist = fabs(limits_[i][j]-joint_position_(i));
          if (dist < closest_limit_dist) {
            closest_limit_dist = dist;
            closest_limit_idx = j;
          }
          dist = fabs(limits_[i][j+1]-joint_position_(i));
          if (dist < closest_limit_dist) {
            closest_limit_dist = dist;
            closest_limit_idx = j;
          }
      }

      if (closest_limit_idx < 0) {
        // Something really bad happend
        RTT::Logger::In in("JointLimitAvoidance::updateHook");
        error();
        Logger::log() << Logger::Error << "could not find violated limit" << Logger::endl;
        return;
      }
      limit_ok = true;
      lo_limit = limits_[i][closest_limit_idx];
      up_limit = limits_[i][closest_limit_idx+1];

      //RTT::Logger::In in("JointLimitAvoidance::updateHook");
      //error();
      //Logger::log() << Logger::Error << "joint " << i << " position in not within limits: " << joint_position_(i) << Logger::endl;
      //return;
    }
    joint_torque_command_(i) = jointLimitTrq(up_limit,
                               lo_limit, limit_range_[i], max_trq_[i],
                               joint_position_(i));
    if (abs(joint_torque_command_(i)) > 0.001) {
      k_(i) = max_trq_[i]/limit_range_[i];
    } else {
      k_(i) = 0.001;
    }
  }

  if (port_mass_matrix_.read(m_) != RTT::NewData) {
    RTT::Logger::In in("JointLimitAvoidance::updateHook");
    error();
    Logger::log() << Logger::Error << "could not read port: " << port_mass_matrix_.getName() << Logger::endl;
    return;
  }

  tmpNN_ = k_.asDiagonal();
  es_.compute(tmpNN_, m_);
  q_ = es_.eigenvectors().inverse();
  k0_ = es_.eigenvalues();

  tmpNN_ = k0_.cwiseSqrt().asDiagonal();

  d_.noalias() = 2.0 * q_.adjoint() * 0.7 * tmpNN_ * q_;

  joint_torque_command_.noalias() -= d_ * joint_velocity_;

  joint_torque_command_.noalias() += nullspace_torque_command_;

  port_joint_torque_command_.write(joint_torque_command_);
}

template <unsigned DOFS>
bool JointLimitAvoidance<DOFS>::startHook() {
  RESTRICT_ALLOC;
  return true;
}

template <unsigned DOFS>
void JointLimitAvoidance<DOFS>::stopHook() {
  for (int i = 0; i < joint_torque_command_.size(); i++) {
    joint_torque_command_(i) = 0.0;
  }
  port_joint_torque_command_.write(joint_torque_command_);
  UNRESTRICT_ALLOC;
}

template <unsigned DOFS>
double JointLimitAvoidance<DOFS>::jointLimitTrq(double hl, double ll, double ls,
    double r_max, double q) {
  if (q > (hl - ls)) {
    double torque = -1 * ((q - hl + ls) / ls) * ((q - hl + ls) / ls) * r_max;
    if (torque < -r_max) {
      torque = -r_max;
    }
    return torque;
  } else if (q < (ll + ls)) {
    double torque = ((ll + ls - q) / ls) * ((ll + ls - q) / ls) * r_max;
    if (torque > r_max) {
      torque = r_max;
    }
    return torque;
  } else {
    return 0.0;
  }
}
#endif  // JOINT_LIMIT_AVOIDANCE_H_
