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

public:
    static constexpr auto QUEUE_LENGTH = (N_FRAMES * 5);

    explicit EcCanQueue(RTT::TaskContext* owner)
        :   controller_common::CanQueueService(owner)
        ,   port_rx_queue_in_("rx_queue_INPORT")
        ,   read_frames_count_(0)
        ,   read_frames_ptr_(0)
        ,   port_tx_out_("tx_OUTPORT")
    {
        for (int i = 0; i < QUEUE_LENGTH; ++i) {
            read_frames_valid_[i] = false;
        }

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
            RTT::Logger::In in("EcCanQueue::readQueue");
            Logger::log() << Logger::Error << "wrong frames read msgcount: " << msgcount << Logger::endl;
            return false;
        }

        for (uint16_t i = 0; i < msgcount; ++i) {
            if (!deserialize(rxdata + 6 + i * 10, read_frames_[read_frames_ptr_])) {
                RTT::Logger::In in("EcCanQueue::readQueue");
                Logger::log() << Logger::Error << "could not deserialize CAN frame: " << Logger::endl;
                return false;
            }

            bool match = false;
            if (filters_.size() == 0) {
                match = true;
            }

            for (int j = 0; j < filters_.size(); ++j) {
                if ((read_frames_[read_frames_ptr_].id & filters_[j].second)
                    == (filters_[j].first & filters_[j].second))
                {
                    match = true;
                    break;
                }
            }

            if (match) {
                read_frames_valid_[read_frames_ptr_] = true;
                read_frames_ptr_ = (read_frames_ptr_+1) % QUEUE_LENGTH;
                if (read_frames_count_ < QUEUE_LENGTH) {
                    ++read_frames_count_;
                }
            }
        }

        return true;
    }

    bool readReply(uint16_t can_id, uint16_t &dlc, int8_t *data_out) {
#ifdef DBG_PRINT_BUFFER_LOAD
        // one in tousand reads, print queue length
        if ((dbg_counter_ % 1000) == 0) {
            int frames_count = 0;
            for (uint16_t i = 0; i < QUEUE_LENGTH; ++i) {
                // Search for the oldest matching frame
                int frame_idx = (read_frames_ptr_ + i) % QUEUE_LENGTH;;
                if (read_frames_valid_[frame_idx] == false) {
                    continue;
                }
                frames_count++;
            }
            // TODO: remove this:
            printf("%d: frames count: %d\n", dbg_id_, frames_count);
        }
        dbg_counter_++;
#endif  // DBG_PRINT_BUFFER_LOAD
        for (uint16_t i = 0; i < QUEUE_LENGTH; ++i) {
            // Search for the oldest matching frame
            int frame_idx = (read_frames_ptr_ + i) % QUEUE_LENGTH;;
            if (read_frames_valid_[frame_idx] == false) {
                continue;
            }

            if ((read_frames_[frame_idx].id == can_id) && read_frames_valid_[frame_idx]) {
                dlc = read_frames_[frame_idx].dlc;
                for (uint16_t j = 0; j < dlc; ++j) {
                    data_out[j] = read_frames_[frame_idx].data[j];
                }
                read_frames_valid_[frame_idx] = false;
                return true;
            }
        }

        return false;
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

    RTT::InputPort<boost::array<int8_t, N_FRAMES*10+6>> port_rx_queue_in_;
    boost::array<can_frame, QUEUE_LENGTH> read_frames_;
    boost::array<bool, QUEUE_LENGTH> read_frames_valid_;
    int read_frames_count_, read_frames_ptr_;
    std::vector<std::pair<uint32_t, uint32_t>> filters_;
    boost::array<int8_t, N_FRAMES*10+6> rx_queue_in_;

    RTT::OutputPort<can_frame > port_tx_out_;
};

}   // namespace ec_can_queue
}   // namespace controller_common

#endif  // CONTROLLER_COMMON_EC_CAN_QUEUE_H__

