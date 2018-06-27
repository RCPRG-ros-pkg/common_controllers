// Copyright 2014 WUT

#ifndef VECTOR_CONCATE_H_
#define VECTOR_CONCATE_H_

#include "Eigen/Dense"

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <vector>
#include <string>

#include <boost/static_assert.hpp>

template <int n1, int n2, int n3, int n4>
class VectorConcate : public RTT::TaskContext {
  BOOST_STATIC_ASSERT(n1 > 0);
  BOOST_STATIC_ASSERT(n2 > 0);
 public:
  explicit VectorConcate(const std::string & name) :
    RTT::TaskContext(name),
    port_input1_("In0"),
    port_input2_("In1"),
    port_input3_("In2"),
    port_input4_("In3"),
    port_output_("Out", true) {

    this->ports()->addPort(port_input1_);
    this->ports()->addPort(port_input2_);
    this->ports()->addPort(port_input3_);
    this->ports()->addPort(port_input4_);
    this->ports()->addPort(port_output_);
  }

  ~VectorConcate() {
  }

  void updateHook() {
    if ((n1 == 0 || port_input1_.read(input1_) == RTT::NewData) &&
        (n2 == 0 || port_input2_.read(input2_) == RTT::NewData) &&
        (n3 == 0 || port_input3_.read(input3_) == RTT::NewData) &&
        (n4 == 0 || port_input4_.read(input4_) == RTT::NewData)) {

      output_.template block<n1, 1>(0, 0) = input1_;
      output_.template block<n2, 1>(n1, 0) = input2_;
      if (n3 != 0) {
        output_.template block<n3, 1>(n1+n2, 0) = input3_;
      }
      if (n4 != 0) {
        output_.template block<n4, 1>(n1+n2+n3, 0) = input4_;
      }
      port_output_.write(output_);
    }
  }

 private:

  typedef Eigen::Matrix<double, n1, 1> VectorIn1;
  typedef Eigen::Matrix<double, n2, 1> VectorIn2;
  typedef Eigen::Matrix<double, n3, 1> VectorIn3;
  typedef Eigen::Matrix<double, n4, 1> VectorIn4;
  RTT::InputPort<VectorIn1 > port_input1_;
  RTT::InputPort<VectorIn2 > port_input2_;
  RTT::InputPort<VectorIn3 > port_input3_;
  RTT::InputPort<VectorIn4 > port_input4_;
  VectorIn1 input1_;
  VectorIn2 input2_;
  VectorIn3 input3_;
  VectorIn4 input4_;

  typedef Eigen::Matrix<double, n1+n2+n3+n4, 1> VectorOut;
  RTT::OutputPort<VectorOut > port_output_;
  VectorOut output_;
};


#endif  // VECTOR_CONCATE_H_

