/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group,
 Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the Warsaw University of Technology nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * tf_publisher.cpp
 *
 *  Created on: 16 jul 2014
 *      Author: dseredyn
 */

#include "tf_publisher.h"

#include <string>

#include "rtt_rosclock/rtt_rosclock.h"

using namespace RTT;

TfPublisher::TfPublisher(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      N_(0) {
  this->ports()->addPort("OutTf", port_out_tf_);
  this->addProperty("frame_ids", frame_ids_);
  this->addProperty("child_frame_ids", child_frame_ids_);
}

TfPublisher::~TfPublisher() {
}

bool TfPublisher::configureHook() {
  if (frame_ids_.size() != child_frame_ids_.size()) {
    Logger::log() << Logger::Error << "Wrong size of input vectors"
                         << Logger::endl;
    return false;
  }

  N_ = frame_ids_.size();

  if (N_ == 0) {
    Logger::log() << Logger::Error << "Input vectors is null"
                         << Logger::endl;
    return false;
  }

  message_.transforms.reserve(N_);
  port_in_.resize(N_);

  for (size_t i = 0; i < N_; i++) {
    char name[30];
    snprintf(name, sizeof(name), "In%zu", i);
    port_in_[i] = new RTT::InputPort<geometry_msgs::Pose>();
    this->ports()->addPort(name, *port_in_[i]);
  }

  return true;
}

bool TfPublisher::startHook() {
  return true;
}

void TfPublisher::updateHook() {
  size_t tf_count = 0;
  message_.transforms.resize(N_);   // the space was reserved in configureHook
  for (size_t i = 0; i < N_; i++) {
    geometry_msgs::Pose pose;
    if (port_in_[i]->read(pose) == RTT::NewData) {
      message_.transforms[tf_count].header.stamp = rtt_rosclock::host_now();
      message_.transforms[tf_count].header.frame_id = frame_ids_[i];
      message_.transforms[tf_count].child_frame_id = child_frame_ids_[i];
      message_.transforms[tf_count].transform.translation.x = pose.position.x;
      message_.transforms[tf_count].transform.translation.y = pose.position.y;
      message_.transforms[tf_count].transform.translation.z = pose.position.z;
      message_.transforms[tf_count].transform.rotation = pose.orientation;
      ++tf_count;
    }
  }
  message_.transforms.resize(tf_count);     // the space was reserved in configureHook

  port_out_tf_.write(message_);
}

