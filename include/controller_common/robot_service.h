// Copyright 2014 WUT

/*
 * robot_service.h
 *
 *  Created on: 27 sty 2014
 *      Author: konradb3
 */

#ifndef ROBOT_SERVICE_H_
#define ROBOT_SERVICE_H_

#include "Eigen/Dense"

#include <rtt/RTT.hpp>
#include <rtt/Service.hpp>
#include <rtt/Logger.hpp>
#include <rtt/plugin/PluginLoader.hpp>

namespace controller_common {

template <unsigned DOFS, unsigned EFFECTORS>
class RobotService: public RTT::Service {
 public:
  explicit RobotService(RTT::TaskContext* owner, const std::string& service_name) :
      RTT::Service(service_name, owner) {
    this->addOperation("jacobian", &RobotService::jacobian, this,
                        RTT::ClientThread);
    this->addOperation("fkin", &RobotService::fkin, this, RTT::ClientThread);
    this->addOperation("dofs", &RobotService::getNumberOfDofs, this,
                        RTT::ClientThread);
    this->addOperation("effectors", &RobotService::getNumberOfEffectors, this,
                        RTT::ClientThread);
  }

  typedef Eigen::Matrix<double, EFFECTORS*6, DOFS> Jacobian;
  typedef Eigen::Matrix<double, DOFS, 1> Joints;
  typedef Eigen::Matrix<double, 4, 1> ToolMass;
  typedef Eigen::Matrix<double, 7, 1> Tool;

  virtual void jacobian(Jacobian &, const Joints &, const Tool*) = 0;
  virtual void fkin(Eigen::Affine3d *, const Joints &, const Tool*) = 0;
  virtual int getNumberOfDofs(void) = 0;
  virtual int getNumberOfEffectors(void) = 0;
};
}  // namespace controller_common

#endif  // ROBOT_SERVICE_H_
