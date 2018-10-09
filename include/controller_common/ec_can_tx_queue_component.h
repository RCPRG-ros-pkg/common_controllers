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

#ifndef CONTROLLER_COMMON_EC_CAN_TX_QUEUE_COMPONENT_H__
#define CONTROLLER_COMMON_EC_CAN_TX_QUEUE_COMPONENT_H__

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include "ec_can_queue.h"

using namespace RTT;

namespace controller_common {

namespace ec_can_queue {

template <size_t N_FRAMES >
class CanQueueTxComponent: public RTT::TaskContext {
public:

    explicit CanQueueTxComponent(const std::string &name) 
        : TaskContext(name)
        , port_tx_in_("tx_INPORT")
        , port_rx_queue_in_("rx_queue_INPORT")
        , port_tx_queue_out_("tx_queue_OUTPORT")
        , rxCount_prev_(0)
        , txCount_prev_(0)
        , invert_rx_tx_(false)
    {
        this->ports()->addPort(port_tx_in_);
        this->ports()->addPort(port_rx_queue_in_);
        this->ports()->addPort(port_tx_queue_out_);

        addProperty("invert_rx_tx", invert_rx_tx_);

        for (int i = 0; i < N_FRAMES*10+6; ++i) {
            rx_queue_in_[i] = 0;
            tx_queue_out_[i] = 0;
        }
    }

    bool startHook() {
        return true;
    }

    void updateHook() {
        int msgs_count = 0;

        if (port_rx_queue_in_.read(rx_queue_in_) == RTT::NewData) {
            uint16_t txCount;
            if (invert_rx_tx_) {
                txCount = *reinterpret_cast<uint16_t* >(rx_queue_in_.data()+2);
            }
            else {
                txCount = *reinterpret_cast<uint16_t* >(rx_queue_in_.data()+0);
            }

            if (txCount_prev_ != txCount) {
                Logger::log() << Logger::Warning << "lost tx counter: " << txCount_prev_ << " " << txCount << Logger::endl;
                txCount_prev_ = txCount;
            }
            ++txCount_prev_;
            can_frame fr;
            while (port_tx_in_.read(fr) == RTT::NewData) {
                if (msgs_count >= N_FRAMES) {
                    RTT::Logger::In in("CanQueueTxComponent::updateHook");
                    Logger::log() << Logger::Error << "queue is overloaded" << Logger::endl;
                    break;
                }
                serialize(fr, tx_queue_out_.data() + 6 + msgs_count * 10);
                ++msgs_count;
            }

            *reinterpret_cast<uint16_t* >(tx_queue_out_.data()+4) = msgs_count;

            uint16_t rxCount;
            if (invert_rx_tx_) {
                *reinterpret_cast<uint16_t* >(tx_queue_out_.data()+2) = txCount_prev_;
                rxCount = *reinterpret_cast<uint16_t* >(rx_queue_in_.data()+0);
            }
            else {
                *reinterpret_cast<uint16_t* >(tx_queue_out_.data()+0) = txCount_prev_;
                rxCount = *reinterpret_cast<uint16_t* >(rx_queue_in_.data()+2);
            }
            if (rxCount_prev_ != uint16_t(rxCount-1)) {
                Logger::log() << Logger::Warning << "lost rx counter: " << rxCount_prev_ << " " << uint16_t(rxCount-1) << Logger::endl;
                rxCount_prev_ = rxCount-1;
            }
            ++rxCount_prev_;  // TODO: verify this on HW
            if (invert_rx_tx_) {
                *reinterpret_cast<uint16_t* >(tx_queue_out_.data()+0) = rxCount_prev_;
            }
            else {
                *reinterpret_cast<uint16_t* >(tx_queue_out_.data()+2) = rxCount_prev_;
            }
        }
        else {
            // do nothing
        }

        port_tx_queue_out_.write(tx_queue_out_);
    }

private:
    bool serialize(const can_frame &frame, int8_t *ptr) {
        if (frame.dlc > 8) {
            return false;
        }
        uint8_t *uptr = reinterpret_cast<uint8_t* >(ptr);
        uptr[0] = (frame.id>>3)&0xFF;
        uptr[1] = ((frame.id&0x07)<<5) | (frame.dlc&0x0F);
        for (int i = 0; i < frame.dlc; ++i) {
            ptr[i + 2] = frame.data[i];
        }
        return true;
    }

    RTT::InputPort<can_frame > port_tx_in_;
    RTT::InputPort<boost::array<int8_t, N_FRAMES*10+6 > > port_rx_queue_in_;
    RTT::OutputPort<boost::array<int8_t, N_FRAMES*10+6 > > port_tx_queue_out_;
    boost::array<int8_t, N_FRAMES*10+6 > rx_queue_in_;
    boost::array<int8_t, N_FRAMES*10+6 > tx_queue_out_;
    uint16_t rxCount_prev_;
    uint16_t txCount_prev_;
    bool invert_rx_tx_;
};

}   // namespace ec_can_queue

}   // namespace controller_common

#endif  // CONTROLLER_COMMON_EC_CAN_TX_QUEUE_COMPONENT_H__

