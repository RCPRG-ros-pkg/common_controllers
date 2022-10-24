// Copyright 2014 WUT
/*
 * cartesian_impedance.h
 *
 *  Created on: 26 sty 2014
 *      Author: konrad
 */

#ifndef CARTESIAN_IMPEDANCE_H_
#define CARTESIAN_IMPEDANCE_H_

#include <cstring>

#include <vector>
#include <string>

#include <boost/array.hpp>

#include "Eigen/Dense"

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"

#include "controller_common/robot.h"

#include "geometry_msgs/Pose.h"
#include "cartesian_trajectory_msgs/CartesianImpedance.h"

#include "eigen_conversions/eigen_msg.h"

#include "fabric_logger/fabric_logger.h"

using namespace RTT;
using fabric_logger::FabricLoggerInterfaceRtPtr;
using fabric_logger::FabricLogger;

template <unsigned DOFS, unsigned EFFECTORS>
class CartesianImpedance: public RTT::TaskContext {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CartesianImpedance(const std::string &name);

  bool configureHook();

  bool startHook();

  void stopHook();

  void updateHook();

  typedef controller_common::Robot<DOFS, EFFECTORS> Robot;

  typedef Eigen::Matrix<double, EFFECTORS * 6, EFFECTORS * 6> MatrixEE;
  typedef Eigen::Matrix<double, DOFS, DOFS> Inertia;
  typedef Eigen::Matrix<double, DOFS, DOFS> MatrixDD;
  typedef Eigen::Matrix<double, EFFECTORS * 6, DOFS> Jacobian;
  typedef Eigen::Matrix<double, DOFS, EFFECTORS * 6> JacobianT;
  typedef Eigen::Matrix<double, DOFS, 1> Joints;
  typedef Eigen::Matrix<double, EFFECTORS * 6, 1> Stiffness;
  typedef Eigen::Matrix<double, EFFECTORS * 6, 1> Spring;
  typedef Eigen::Matrix<double, EFFECTORS * 6, 1> Force;
  typedef Eigen::Matrix<double, 4, 1> ToolMass;
  typedef Eigen::Matrix<double, 7, 1> Tool;

 private:
  RTT::InputPort<Joints> port_joint_position_;
  RTT::InputPort<Joints> port_joint_velocity_;
  RTT::InputPort<Inertia> port_mass_matrix_inv_;

  boost::array<RTT::InputPort<geometry_msgs::Pose>*, EFFECTORS > port_cartesian_position_command_;
  boost::array<RTT::OutputPort<geometry_msgs::Pose>*, EFFECTORS > port_cartesian_position_;
  boost::array<RTT::InputPort<geometry_msgs::Pose>*, EFFECTORS > port_tool_position_command_;
  boost::array<RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedance>*, EFFECTORS > port_cartesian_impedance_command_;

  RTT::InputPort<Joints> port_nullspace_torque_command_;

  RTT::OutputPort<Joints> port_joint_torque_command_;

  Joints joint_position_;
  Joints joint_velocity_;

  Joints joint_torque_command_;
  Joints nullspace_torque_command_;

  boost::shared_ptr<Robot> robot_;

  Eigen::GeneralizedSelfAdjointEigenSolver<MatrixEE> es_;

  Eigen::PartialPivLU<Inertia> lu_;
  Eigen::PartialPivLU<MatrixEE> luKK_;

  boost::array<Tool, EFFECTORS> tools;

  boost::array<Eigen::Affine3d, EFFECTORS> r_cmd;

  Stiffness Kc, Dxi;
  Jacobian J;
  JacobianT JT, Ji;
  Inertia M, Mi, P;
  MatrixEE A, Q, Dc;
  Stiffness K0;
  Spring p;
  Force F;
  MatrixEE tmpKK_, tmpKK2_;
  MatrixDD tmpNN_;
  Jacobian tmpKN_;
  JacobianT tmpNK_;
  Stiffness tmpK_;

  bool first_step_;

  FabricLoggerInterfaceRtPtr m_fabric_logger;

  // Parameters
  bool enable_rt_logging_;

  std::string service_name_;
};

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

template <unsigned DOFS, unsigned EFFECTORS>
  CartesianImpedance<DOFS, EFFECTORS>::CartesianImpedance(const std::string &name)
  : RTT::TaskContext(name, PreOperational)
  , port_joint_torque_command_("JointTorqueCommand_OUTPORT", false)
  , port_joint_position_("JointPosition_INPORT")
  , port_joint_velocity_("JointVelocity_INPORT")
  , port_mass_matrix_inv_("MassMatrixInv_INPORT")
  , port_nullspace_torque_command_("NullSpaceTorqueCommand_INPORT")
  , m_fabric_logger( FabricLogger::createNewInterfaceRt( std::string("CartesianImpedance: ") + name, 1000000) )
  , enable_rt_logging_(false)
  {
    this->ports()->addPort(port_joint_position_);
    this->ports()->addPort(port_joint_velocity_);
    this->ports()->addPort(port_mass_matrix_inv_);

    this->ports()->addPort(port_joint_torque_command_);
    this->ports()->addPort(port_nullspace_torque_command_);
    addProperty("enable_rt_logging", enable_rt_logging_);

    this->properties()->addProperty("service_name", service_name_);
  }

template <unsigned DOFS, unsigned EFFECTORS>
  bool CartesianImpedance<DOFS, EFFECTORS>::configureHook() {
    RTT::Logger::In in("CartesianImpedance::configureHook");

    robot_ = this->getProvider<Robot>(service_name_);
    if (!robot_) {
      m_fabric_logger << "ERROR: Unable to load RobotService: " << service_name_
          << FabricLogger::End();
      Logger::log() << Logger::Error << "Unable to load RobotService: " << service_name_
                           << Logger::endl;
      return false;
    }

    if (robot_->dofs() != DOFS) {
      m_fabric_logger << "ERROR: wrong number of DOFs:" << robot_->dofs()
                                                << ", expected " << DOFS << FabricLogger::End();
      Logger::log() << Logger::Error << "wrong number of DOFs: " << robot_->dofs()
                                                      << ", expected " << DOFS << Logger::endl;
      return false;
    }

    if (robot_->effectors() != EFFECTORS) {
      m_fabric_logger << "ERROR: wrong number of effectors: " << robot_->effectors()
                                          << ", expected " << EFFECTORS << FabricLogger::End();
      Logger::log() << Logger::Error << "wrong number of effectors: " << robot_->effectors()
                                                    << ", expected " << EFFECTORS << Logger::endl;
      return false;
    }

//    port_cartesian_position_command_.resize(K);
//    port_cartesian_position_.resize(K);
//    port_cartesian_impedance_command_.resize(K);
//    port_tool_position_command_.resize(K);

    for (size_t i = 0; i < EFFECTORS; i++) {
      char name[30];
      snprintf(name, sizeof(name), "CartPositionCommand%zu_INPORT", i);
      port_cartesian_position_command_[i] = new RTT::InputPort<geometry_msgs::Pose>(name);
      this->ports()->addPort(*port_cartesian_position_command_[i]);

      snprintf(name, sizeof(name), "CartesianPosition%zu_OUTPORT", i);
      port_cartesian_position_[i] = new RTT::OutputPort<geometry_msgs::Pose>(name, false);
      this->ports()->addPort(*port_cartesian_position_[i]);
      port_cartesian_position_[i]->keepLastWrittenValue(false);

      snprintf(name, sizeof(name), "ToolPositionCommand%zu_INPORT", i);
      port_tool_position_command_[i] = new RTT::InputPort<geometry_msgs::Pose>(name);
      this->ports()->addPort(*port_tool_position_command_[i]);

      snprintf(name, sizeof(name), "CartImpedanceCommand%zu_INPORT", i);
      port_cartesian_impedance_command_[i] = new RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedance>(name);
      this->ports()->addPort(*port_cartesian_impedance_command_[i]);
    }

//    tools.resize(K);
//    r_cmd.resize(K);

//    joint_position_.resize(N);
//    joint_velocity_.resize(N);
//    joint_torque_command_.resize(N);
//    nullspace_torque_command_.resize(N),
//                                     Kc.resize(K * 6);
//    Dxi.resize(K * 6);
//    J.resize(K * 6, N);
//    JT.resize(N, K * 6);
//    Ji.resize(N, K * 6);
//    M.resize(N, N);
//    Mi.resize(N, N);
//    P.resize(N, N);
//    A.resize(K * 6, K * 6);
//    Q.resize(K * 6, K * 6);
//    Dc.resize(K * 6, K * 6);
//    K0.resize(K * 6);
//    p.resize(K * 6);
//    F.resize(K * 6);

//    tmpNK_.resize(N, K * 6);
//    tmpKK_.resize(K * 6, K * 6);
//    tmpKK2_.resize(K * 6, K * 6);
//    tmpK_.resize(K * 6);
//    tmpNN_.resize(N, N);
//    tmpKN_.resize(K * 6, N);

//    lu_ = Eigen::PartialPivLU<Eigen::MatrixXd>(N);
//    luKK_ = Eigen::PartialPivLU<Eigen::MatrixXd>(K*6);
//    es_ = Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd>(K * 6);

    port_joint_torque_command_.setDataSample(joint_torque_command_);

    return true;
  }

template <unsigned DOFS, unsigned EFFECTORS>
  bool CartesianImpedance<DOFS, EFFECTORS>::startHook() {
    RESTRICT_ALLOC;

    first_step_ = true;

    nullspace_torque_command_.setZero();

    UNRESTRICT_ALLOC;
    return true;
  }

template <unsigned DOFS, unsigned EFFECTORS>
  void CartesianImpedance<DOFS, EFFECTORS>::stopHook() {
    for (size_t i = 0; i < joint_torque_command_.size(); i++) {
      joint_torque_command_(i) = 0.0;
    }
    port_joint_torque_command_.write(joint_torque_command_);
  }

template <unsigned DOFS, unsigned EFFECTORS>
  void CartesianImpedance<DOFS, EFFECTORS>::updateHook() {

    RESTRICT_ALLOC;

    if (first_step_) {
      for (size_t i = 0; i < EFFECTORS; i++) {
        Kc(i * 6 + 0) = 1500;
        Kc(i * 6 + 1) = 1500;
        Kc(i * 6 + 2) = 1500;
        Kc(i * 6 + 3) = 150;
        Kc(i * 6 + 4) = 150;
        Kc(i * 6 + 5) = 150;

        Dxi(i * 6 + 0) = 0.35;
        Dxi(i * 6 + 1) = 0.35;
        Dxi(i * 6 + 2) = 0.35;
        Dxi(i * 6 + 3) = 0.35;
        Dxi(i * 6 + 4) = 0.35;
        Dxi(i * 6 + 5) = 0.35;

        geometry_msgs::Pose pos;
        if (port_tool_position_command_[i]->read(pos) == RTT::NewData) {
          tools[i](0) = pos.position.x;
          tools[i](1) = pos.position.y;
          tools[i](2) = pos.position.z;

          tools[i](3) = pos.orientation.w;
          tools[i](4) = pos.orientation.x;
          tools[i](5) = pos.orientation.y;
          tools[i](6) = pos.orientation.z;
        }
        else {
          m_fabric_logger << "ERROR: could not read port \'"
                      << port_tool_position_command_[i]->getName() << "\'" << FabricLogger::End();

          //RTT::Logger::In in("CartesianImpedance::updateHook");
          error();
          //Logger::log() << Logger::Error << "could not read port \'" << port_tool_position_command_[i]->getName() << "\'" << Logger::endl;
          return;
        }
      }
/*
      if (port_joint_position_.read(joint_position_) == RTT::NewData) {
        robot_->fkin(&r_cmd[0], joint_position_, &tools[0]);

        for (size_t i = 0; i < EFFECTORS; i++) {
          geometry_msgs::Pose pos;
          tf::poseEigenToMsg(r_cmd[i], pos);
          port_cartesian_position_[i]->write(pos);
        }
      } else {
        error();
        Logger::log() << Logger::Error << "could not read tool position command" << Logger::endl;
        return;
      }
*/
      first_step_ = false;
    }

    // ToolMass toolsM[K];
    Eigen::Affine3d r[EFFECTORS];

    // read inputs
    if (port_joint_position_.read(joint_position_) != RTT::NewData) {
        m_fabric_logger << "ERROR: could not read port \'" << port_joint_position_.getName()
                                                                  << "\'" << FabricLogger::End();

        //RTT::Logger::In in("CartesianImpedance::updateHook");
        error();
        //Logger::log() << Logger::Error << "could not read port \'" << port_joint_position_.getName() << "\'" << Logger::endl;
        return;
    }

    if (!joint_position_.allFinite()) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: joint_position_ contains NaN or inf" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "joint_position_ contains NaN or inf" << Logger::endl;
        return;
    }

    port_joint_velocity_.read(joint_velocity_);
    if (!joint_velocity_.allFinite()) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: joint_velocity_ contains NaN or inf" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "joint_velocity_ contains NaN or inf" << Logger::endl;
        return;
    }

    port_nullspace_torque_command_.read(nullspace_torque_command_);
    if (!nullspace_torque_command_.allFinite()) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: nullspace_torque_command_ contains NaN or inf" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "nullspace_torque_command_ contains NaN or inf" << Logger::endl;
        return;
    }

    //bool rt_logging_output = false;
    for (size_t i = 0; i < EFFECTORS; i++) {
      geometry_msgs::Pose pos;
      if (port_cartesian_position_command_[i]->read(pos) == RTT::NewData) {
        tf::poseMsgToEigen(pos, r_cmd[i]);
        if (enable_rt_logging_) {
          m_fabric_logger << "cmd " << i << " " << pos.position.x << " " << pos.position.y
                          << " " << pos.position.z << " " << pos.orientation.x
                          << " " << pos.orientation.y << " " << pos.orientation.z
                          << " " << pos.orientation.w << FabricLogger::End();
          //rt_logging_output = true;
        }
      }

      if (port_tool_position_command_[i]->read(pos) == RTT::NewData) {
        tools[i](0) = pos.position.x;
        tools[i](1) = pos.position.y;
        tools[i](2) = pos.position.z;

        tools[i](3) = pos.orientation.w;
        tools[i](4) = pos.orientation.x;
        tools[i](5) = pos.orientation.y;
        tools[i](6) = pos.orientation.z;

        if (enable_rt_logging_) {
          m_fabric_logger << "tool " << i << " " << pos.position.x << " " << pos.position.y
                          << " " << pos.position.z << " " << pos.orientation.x
                          << " " << pos.orientation.y << " " << pos.orientation.z
                          << " " << pos.orientation.w << FabricLogger::End();
          //rt_logging_output = true;
        }
      }

      cartesian_trajectory_msgs::CartesianImpedance impedance;
      if (port_cartesian_impedance_command_[i]->read(impedance)
          == RTT::NewData) {
        Kc(i * 6 + 0) = impedance.stiffness.force.x;
        Kc(i * 6 + 1) = impedance.stiffness.force.y;
        Kc(i * 6 + 2) = impedance.stiffness.force.z;
        Kc(i * 6 + 3) = impedance.stiffness.torque.x;
        Kc(i * 6 + 4) = impedance.stiffness.torque.y;
        Kc(i * 6 + 5) = impedance.stiffness.torque.z;

        Dxi(i * 6 + 0) = impedance.damping.force.x;
        Dxi(i * 6 + 1) = impedance.damping.force.y;
        Dxi(i * 6 + 2) = impedance.damping.force.z;
        Dxi(i * 6 + 3) = impedance.damping.torque.x;
        Dxi(i * 6 + 4) = impedance.damping.torque.y;
        Dxi(i * 6 + 5) = impedance.damping.torque.z;
      }
    }

    if (port_mass_matrix_inv_.read(M) != RTT::NewData) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: could not read port \'" << port_mass_matrix_inv_.getName()
                                                                  << "\'" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "could not read port \'" << port_mass_matrix_inv_.getName() << "\'" << Logger::endl;
        return;
    }

    if (!M.allFinite()) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: M contains NaN or inf" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "M contains NaN or inf" << Logger::endl;
        return;
    }

    // calculate robot data
    robot_->jacobian(J, joint_position_, &tools[0]);

    if (!J.allFinite()) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: J contains NaN or inf" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "J contains NaN or inf" << Logger::endl;
        return;
    }

    robot_->fkin(r, joint_position_, &tools[0]);

    if (enable_rt_logging_) {
      for (int i = 0; i < EFFECTORS; i++) {
        Eigen::Quaterniond q( r[i].rotation() );
        Eigen::Vector3d t(r[i].translation());
        m_fabric_logger << "pos " << i << " " << t(0) << " " << t(1)
                        << " " << t(2) << " " << q.x()
                        << " " << q.y() << " " << q.z()
                        << " " << q.w() << FabricLogger::End();
      }
      //rt_logging_output = true;
    }

    //if (enable_rt_logging_ && rt_logging_output) {
    //  m_fabric_logger << FabricLogger::End();
    //}


    JT = J.transpose();
    lu_.compute(M);
    Mi = lu_.inverse();

    if (!Mi.allFinite()) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: Mi contains NaN or inf" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "Mi contains NaN or inf" << Logger::endl;
        return;
    }

    // calculate stiffness component
    for (size_t i = 0; i < EFFECTORS; i++) {
      Eigen::Affine3d tmp;
      tmp = r[i].inverse() * r_cmd[i];

      p(i * 6) = tmp.translation().x();
      p(i * 6 + 1) = tmp.translation().y();
      p(i * 6 + 2) = tmp.translation().z();

      Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(
                                         tmp.rotation());
      p(i * 6 + 3) = quat.x();
      p(i * 6 + 4) = quat.y();
      p(i * 6 + 5) = quat.z();
    }

    F.noalias() = (Kc.array() * p.array()).matrix();
    joint_torque_command_.noalias() = JT * F;

    if (!joint_torque_command_.allFinite()) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: joint_torque_command_ contains NaN or inf" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "joint_torque_command_ contains NaN or inf" << Logger::endl;
        return;
    }

    // calculate damping component

#if 1
    tmpKN_.noalias() = J * Mi;
    A.noalias() = tmpKN_ * JT;
    luKK_.compute(A);
    A = luKK_.inverse();

    tmpKK_ = Kc.asDiagonal();
    UNRESTRICT_ALLOC;
    es_.compute(tmpKK_, A);
    RESTRICT_ALLOC;
    K0 = es_.eigenvalues();
    luKK_.compute(es_.eigenvectors());
    Q = luKK_.inverse();

    tmpKK_ = Dxi.asDiagonal();
    Dc.noalias() = Q.transpose() * tmpKK_;
    tmpKK_ = K0.cwiseSqrt().asDiagonal();
    tmpKK2_.noalias() = Dc *  tmpKK_;
    Dc.noalias() = tmpKK2_ * Q;
    tmpK_.noalias() = J * joint_velocity_;
    F.noalias() = Dc * tmpK_;

    if (!F.allFinite()) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: F contains NaN or inf" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "F contains NaN or inf" << Logger::endl;
        return;
    }

    joint_torque_command_.noalias() -= JT * F;
#else
    UNRESTRICT_ALLOC;
    tmpNN_.noalias() = JT * Kc.asDiagonal() * J;

    es_.compute(tmpNN_, M, Eigen::ComputeEigenvectors | Eigen::BAx_lx);

    K0 = es_.eigenvalues();
    Q = es_.eigenvectors();
    RESTRICT_ALLOC;

    tmpNN_ = K0.cwiseSqrt().asDiagonal();

    UNRESTRICT_ALLOC;
    Dc.noalias() = Q * 0.7 * tmpNN_ * Q.adjoint();
    RESTRICT_ALLOC;
    joint_torque_command_.noalias() -= Dc * joint_velocity_;
#endif

    // calculate null-space component

    tmpKN_.noalias() = J * Mi;
    tmpKK_.noalias() = tmpKN_ * JT;
    luKK_.compute(tmpKK_);
    tmpKK_ = luKK_.inverse();
    tmpNK_.noalias() = Mi * JT;
    Ji.noalias() = tmpNK_ * tmpKK_;

    P.noalias() = Inertia::Identity();
    P.noalias() -=  J.transpose() * A * J * Mi;

    if (!P.allFinite()) {
        //RTT::Logger::In in("CartesianImpedance::updateHook");
        m_fabric_logger << "ERROR: P contains NaN or inf" << FabricLogger::End();
        error();
        //Logger::log() << Logger::Error << "P contains NaN or inf" << Logger::endl;
        return;
    }

    joint_torque_command_.noalias() += P * nullspace_torque_command_;

    // write outputs
    UNRESTRICT_ALLOC;
    port_joint_torque_command_.write(joint_torque_command_);

    if (enable_rt_logging_) {
      m_fabric_logger << "torque ";
      for (int i = 0; i < DOFS; i++) {
        if (i > 0) {
          m_fabric_logger << " ";
        }
        m_fabric_logger << joint_torque_command_(i);
      }
      m_fabric_logger << FabricLogger::End();
    }

    for (size_t i = 0; i < EFFECTORS; i++) {
      geometry_msgs::Pose pos;
      tf::poseEigenToMsg(r[i], pos);
      port_cartesian_position_[i]->write(pos);
    }
  }

#endif  // CARTESIAN_IMPEDANCE_H_

