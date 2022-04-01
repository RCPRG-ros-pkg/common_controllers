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

#ifndef CONTROLLER_COMMON_EC_CAN_QUEUE_H__
#define CONTROLLER_COMMON_EC_CAN_QUEUE_H__

#include <array>

#include <controller_common/can_queue_service.h>
#include <rtt/Logger.hpp>
#include <stdlib.h>

#include "fabric_logger/fabric_logger.h"

using fabric_logger::FabricLoggerInterfaceRtPtr;
using fabric_logger::FabricLogger;

using namespace RTT;

namespace controller_common {
namespace ec_can_queue {

constexpr int N_FRAMES = 10;

//#define DBG_PRINT_BUFFER_LOAD

struct can_frame {
    uint16_t id;
    uint16_t dlc;
    int8_t data[8];
};

class EcCanQueue
    :   public controller_common::CanQueueService {

#ifdef DBG_PRINT_BUFFER_LOAD
    int dbg_counter_;
    int dbg_id_;
#endif  // DBG_PRINT_BUFFER_LOAD

    FabricLoggerInterfaceRtPtr m_fabric_logger;
    int txCount_prev_;
public:
    static constexpr auto QUEUE_LENGTH = (N_FRAMES * 5);

    explicit EcCanQueue(RTT::TaskContext* owner)
        :   controller_common::CanQueueService(owner)
        ,   port_rx_queue_in_("rx_queue_INPORT")
        ,   frames_count_(0)
        ,   txCount_prev_(0)
        ,   port_tx_out_("tx_OUTPORT")
        ,   m_fabric_logger( FabricLogger::createNewInterfaceRt( owner->getName() + "_EcCanQueue", 100000) )
    {
        owner->ports()->addPort(port_rx_queue_in_);
        owner->ports()->addPort(port_tx_out_);

        can_frame fr;
        port_tx_out_.setDataSample(fr);

#ifdef DBG_PRINT_BUFFER_LOAD
        dbg_id_ = rand();
        dbg_counter_ = 0;
#endif  // DBG_PRINT_BUFFER_LOAD
    }

    void initialize(const std::string& dev_name, const std::vector<std::pair<uint32_t, uint32_t>>& filters) {
        filters_ = filters;
    }

    bool send(uint16_t can_id, uint16_t len, const int8_t *data) {
        can_frame fr;
        fr.id = can_id;
        fr.dlc = len;
        for (uint16_t i = 0; i < len; ++i) {
            fr.data[i] = data[i];
        }

        port_tx_out_.write(fr);
        return true;
    }

    bool readQueue() {
        if (port_rx_queue_in_.read(rx_queue_in_, false) != RTT::NewData) {
            return false;
        }

        const auto rxdata = rx_queue_in_.data();
        const auto msgcount = *(uint16_t*)(rxdata + 4);
        if (msgcount > N_FRAMES) {
            m_fabric_logger << "ERROR: wrong frames read msgcount: " << msgcount << FabricLogger::End();

            return false;
        }


        int txCount = *(uint16_t*)(rxdata + 2);
        int rxCount = *(uint16_t*)(rxdata + 0);

        m_fabric_logger << "txCount: " << txCount << ", rxCount: " << rxCount << FabricLogger::End();

        if (txCount != txCount_prev_) {
            txCount_prev_ = txCount;
        }
        else {
            msgcount = 0;
        }

        can_frame frame;
        for (uint16_t i = 0; i < msgcount; ++i) {
            if (!deserialize(rxdata + 6 + i * 10, frame)) {
                m_fabric_logger << "could not deserialize CAN frame" << FabricLogger::End();
                return false;
            }

            bool match = false;
            if (filters_.size() == 0) {
                match = true;
            }

            for (int j = 0; j < filters_.size(); ++j) {
                if ((frame.id & filters_[j].second)
                    == (filters_[j].first & filters_[j].second))
                {
                    match = true;
                    break;
                }
            }

            if (match) {
                // Add the frame
                // Move all older frames by 1 up
                for (int frame_idx = ((frames_count_==QUEUE_LENGTH) ? QUEUE_LENGTH-1 : frames_count_); frame_idx >= 1; --frame_idx) {
                    frames_[frame_idx] = frames_[frame_idx-1];
                }
                frames_[0] = frame;
                if (frames_count_ < QUEUE_LENGTH) {
                    ++frames_count_;
                }
            }
        }

        return true;
    }

    bool readReply(uint16_t can_id, uint16_t &dlc, int8_t *data_out) {
#ifdef DBG_PRINT_BUFFER_LOAD
        // one in tousand reads, print queue length
        if ((dbg_counter_ % 1000) == 0) {
            printf("%s %d: frames count: %d\n", getName().c_str(), dbg_id_, frames_count_);
        }
        dbg_counter_++;
#endif  // DBG_PRINT_BUFFER_LOAD

        // Get the oldest matching frame
        int found_frame_idx = -1;
        for (int frame_idx = frames_count_-1; frame_idx >= 0; --frame_idx) {
            if (frames_[frame_idx].id == can_id) {
                found_frame_idx = frame_idx;
                break;
            }
        }

        if (found_frame_idx >= 0) {
            dlc = frames_[found_frame_idx].dlc;
            for (uint16_t j = 0; j < dlc; ++j) {
                data_out[j] = frames_[found_frame_idx].data[j];
            }

            // Remove the frame
            for (int frame_idx = found_frame_idx; frame_idx < frames_count_-1; ++frame_idx) {
                frames_[frame_idx] = frames_[frame_idx+1];
            }
            if (frames_count_ > 0) {
                --frames_count_;
            }
            return true;
        }

        return false;
    }

    int getFramesCount() const {
        return frames_count_;
    }

private:
    bool deserialize(const int8_t *ptr, can_frame &frame) {
        const auto uptr = (const uint8_t*)(ptr);
        frame.id = (uptr[1] << 3) | ((uptr[0] >> 5) & 0x07);
        frame.dlc = (uptr[0] & 0x0F);
        if (frame.dlc > 8) {
            return false;
        }

        for (int i = 0; i < frame.dlc; ++i) {
            frame.data[i] = ptr[i + 2];
        }

        return true;
    }

    boost::array<can_frame, QUEUE_LENGTH> frames_;
    int frames_count_;

    RTT::InputPort<boost::array<int8_t, N_FRAMES*10+6>> port_rx_queue_in_;
    std::vector<std::pair<uint32_t, uint32_t>> filters_;
    boost::array<int8_t, N_FRAMES*10+6> rx_queue_in_;

    RTT::OutputPort<can_frame > port_tx_out_;
};

}   // namespace ec_can_queue
}   // namespace controller_common

#endif  // CONTROLLER_COMMON_EC_CAN_QUEUE_H__

