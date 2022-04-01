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

#ifndef COMMON_CONTROLLERS_CAN_QUEUE_SERVICE_REQUESTER_H_
#define COMMON_CONTROLLERS_CAN_QUEUE_SERVICE_REQUESTER_H_

#include <string>
#include "rtt/RTT.hpp"

namespace controller_common {

class CanQueueServiceRequester : public RTT::ServiceRequester {
 public:
  explicit CanQueueServiceRequester(RTT::TaskContext *owner)
        : RTT::ServiceRequester("can_queue", owner)
        , initialize("initialize")
        , send("send")
        , readQueue("readQueue")
        , readReply("readReply")
        , readReply("getFramesCount") {
    this->addOperationCaller(initialize);
    this->addOperationCaller(send);
    this->addOperationCaller(readQueue);
    this->addOperationCaller(readReply);
    this->addOperationCaller(getFramesCount);
  }

  RTT::OperationCaller<void(const std::string&, const std::vector<std::pair<uint32_t, uint32_t > >&) > initialize;
  RTT::OperationCaller<bool(uint16_t, uint16_t, const int8_t*) > send;
  RTT::OperationCaller<bool() > readQueue;
  RTT::OperationCaller<bool(uint16_t, uint16_t&, int8_t*) > readReply;
  RTT::OperationCaller<int() > getFramesCount;
};

}  // namespace controller_common

#endif  // COMMON_CONTROLLERS_CAN_QUEUE_SERVICE_REQUESTER_H_
