// Copyright 2014 WUT
/*
 * robot.h
 *
 *  Created on: 26 sty 2014
 *      Author: konrad
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <string>

#include "Eigen/Dense"

#include "rtt/RTT.hpp"

namespace controller_common {

template <unsigned DOFS, unsigned EFFECTORS>
class Robot : public RTT::ServiceRequester {
 public:
  explicit Robot(RTT::TaskContext *owner) :
    RTT::ServiceRequester("robot", owner),
    jacobian("jacobian"),
    fkin("fkin"),
    dofs("dofs"),
    effectors("effectors") {
    this->addOperationCaller(jacobian);
    this->addOperationCaller(fkin);
    this->addOperationCaller(dofs);
    this->addOperationCaller(effectors);
  }

  typedef Eigen::Matrix<double, EFFECTORS*6, DOFS> Jacobian;
  typedef Eigen::Matrix<double, DOFS, 1> Joints;
  typedef Eigen::Matrix<double, 4, 1> ToolMass;
  typedef Eigen::Matrix<double, 7, 1> Tool;

  RTT::OperationCaller<void(Jacobian &, const Joints &, const Tool*)> jacobian;
  RTT::OperationCaller<void(Eigen::Affine3d *, const Joints &, const Tool*)> fkin;
  RTT::OperationCaller<int(void)> dofs;
  RTT::OperationCaller<int(void)> effectors;
};
}  // namespace controller_common

#endif  // ROBOT_H_
