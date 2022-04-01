/*
 Copyright (c) 2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

#ifndef COMMON_CONTROLLERS_CAN_QUEUE_SERVICE_H_
#define COMMON_CONTROLLERS_CAN_QUEUE_SERVICE_H_

#include <rtt/RTT.hpp>

namespace controller_common {

class CanQueueService: public RTT::Service {
 public:
  explicit CanQueueService(RTT::TaskContext* owner)
        : RTT::Service("can_queue", owner)
  {
    this->addOperation("initialize", &CanQueueService::initialize, this, RTT::ClientThread);
    this->addOperation("send", &CanQueueService::send, this, RTT::ClientThread);
    this->addOperation("readQueue", &CanQueueService::readQueue, this, RTT::ClientThread);
    this->addOperation("readReply", &CanQueueService::readReply, this, RTT::ClientThread);
    this->addOperation("getFramesCount", &CanQueueService::getFramesCount, this, RTT::ClientThread);
  }

  virtual void initialize(const std::string& dev_name, const std::vector<std::pair<uint32_t, uint32_t>>& filters) = 0;
  virtual bool send(uint16_t can_id, uint16_t len, const int8_t *data) = 0;
  virtual bool readQueue() = 0;
  virtual bool readReply(uint16_t can_id, uint16_t &dlc, int8_t *data) = 0;
  virtual int getFramesCount() const = 0;
};
}  // namespace controller_common

#endif  // COMMON_CONTROLLERS_CAN_QUEUE_SERVICE_H_
