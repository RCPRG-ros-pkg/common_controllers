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

#include <stdlib.h>

#include "ec_can_queue.h"

using namespace RTT;

namespace controller_common {
namespace ec_can_queue {

class CanQueueTxComponent
    :   public RTT::TaskContext {
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
        //printf("ec_can_tx_queue_component invert_rx_tx_: %d\n", (int)invert_rx_tx_);
        return true;
    }

    void handleRxData()
    {
        const auto rxdata = rx_queue_in_.data();

        uint16_t rxCount;
        uint16_t txCount;
        if (invert_rx_tx_) {
            txCount = *(uint16_t*)(rxdata + 2);
            rxCount = *(uint16_t*)(rxdata + 0);
        }
        else {
            txCount = *(uint16_t*)(rxdata + 0);
            rxCount = *(uint16_t*)(rxdata + 2);
        }

        if (rxCount != rxCount_prev_) {
            // new data arrived
            rxCount_prev_ = rxCount;
            // TODO: read new data
        }

        auto txdata = tx_queue_out_.data();

        int msgs_count = 0;
        if (txCount_prev_ == txCount) {
            // If we are here, it means that CAN is ready to get new data from us
            can_frame fr;
            while (port_tx_in_.read(fr) == RTT::NewData) {
                if (msgs_count >= N_FRAMES) {
                    RTT::Logger::In in("CanQueueTxComponent::updateHook");
                    Logger::log() << Logger::Error << "queue is overloaded" << Logger::endl;
                    break;
                }

                if (!serialize(fr, txdata + 6 + msgs_count*10)) {
                    RTT::Logger::In in("CanQueueTxComponent::updateHook");
                    Logger::log() << Logger::Error << "could not serialize CAN frame" << Logger::endl;
                    break;
                }

                ++msgs_count;
            }

            if (msgs_count > 0) {
                ++txCount_prev_;
            }
        }

        if (invert_rx_tx_) {
            *(uint16_t*)(txdata + 2) = txCount_prev_;
            *(uint16_t*)(txdata + 0) = rxCount_prev_;
        }
        else {
            *(uint16_t*)(txdata + 0) = txCount_prev_;
            *(uint16_t*)(txdata + 2) = rxCount_prev_;
        }

        *(uint16_t*)(txdata + 4) = msgs_count;
        if (msgs_count > 0) {
//            printf("! ec_can_tx_queue_component msgs_count: %d tx: %u r_tx: %u rx: %u r_rx: %u, %d %d %d %d %d %d %d\n", msgs_count, txCount_prev_, txCount, rxCount_prev_, rxCount, (int)tx_queue_out_.data()[0], (int)tx_queue_out_.data()[1], (int)tx_queue_out_.data()[2], (int)tx_queue_out_.data()[3], (int)tx_queue_out_.data()[4], (int)tx_queue_out_.data()[5], (int)tx_queue_out_.data()[6]);
        }
    }

    void updateHook() {
        if (port_rx_queue_in_.read(rx_queue_in_) == RTT::NewData) 
        {}
        handleRxData();

        port_tx_queue_out_.write(tx_queue_out_);
    }

private:
    bool serialize(const can_frame &frame, int8_t *ptr) {
        if (frame.dlc > 8) {
            return false;
        }

        auto uptr = (uint8_t*)(ptr);
        uptr[1] = (frame.id >> 3) & 0xFF;
        uptr[0] = ((frame.id & 0x07) << 5) | (frame.dlc & 0x0F);
        for (int i = 0; i < frame.dlc; ++i) {
            ptr[i + 2] = frame.data[i];
        }

        return true;
    }

    RTT::InputPort<can_frame> port_tx_in_;
    RTT::InputPort<boost::array<int8_t, N_FRAMES*10+6>> port_rx_queue_in_;
    RTT::OutputPort<boost::array<int8_t, N_FRAMES*10+6>> port_tx_queue_out_;
    uint16_t rxCount_prev_;
    uint16_t txCount_prev_;
    bool invert_rx_tx_;
    boost::array<int8_t, N_FRAMES*10+6> rx_queue_in_;
    boost::array<int8_t, N_FRAMES*10+6> tx_queue_out_;
};

}   // namespace ec_can_queue
}   // namespace controller_common

#endif  // CONTROLLER_COMMON_EC_CAN_TX_QUEUE_COMPONENT_H__
