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

#include <boost/array.hpp>
#include <controller_common/can_queue_service.h>
#include <rtt/Logger.hpp>

namespace controller_common {

namespace ec_can_queue {

struct can_frame {
    uint16_t id;
    uint16_t dlc;
    int8_t data[8];
};

template <size_t N_FRAMES >
class EcCanQueue: public controller_common::CanQueueService {
public:
    explicit EcCanQueue(RTT::TaskContext* owner)
            : controller_common::CanQueueService(owner)
            , port_rx_queue_in_("rx_queue_INPORT")
            , read_frames_count_(0)
            , port_tx_out_("tx_OUTPORT")
    {
        owner->ports()->addPort(port_rx_queue_in_);
        owner->ports()->addPort(port_tx_out_);
        can_frame fr;
        port_tx_out_.setDataSample(fr);
    }

    void initialize(const std::string& dev_name, const std::vector<std::pair<uint32_t, uint32_t > >& filters) {
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

    bool readReply(uint16_t can_id, uint16_t &dlc, int8_t *data_out) {
        if (port_rx_queue_in_.read(rx_queue_in_, false) == RTT::NewData) {
            uint32_t count = rx_queue_in_[4] | (rx_queue_in_[5]<<8);
            if (count > N_FRAMES) {
                RTT::Logger::In in("EcCanQueue::readReply");
                RTT::log(RTT::Error) << "wrong frames read count: " << count << RTT::endlog();
                return false;
            }
            if (read_frames_count_ >= N_FRAMES*5) {
                RTT::Logger::In in("EcCanQueue::readReply");
                RTT::log(RTT::Error) << "wrong frames total count: " << read_frames_count_ << RTT::endlog();
                return false;
            }
            for (uint16_t i = 0; i < count; ++i) {
                can_frame fr;
                if (!deserialize(rx_queue_in_.data() + 6 + i * 10, fr)) {
                    RTT::Logger::In in("EcCanQueue::readReply");
                    RTT::log(RTT::Error) << "could not deserialize CAN frame: " << RTT::endlog();
                    return false;
                }
            }
            for (uint16_t i = 0; i < count; ++i) {
                if (!deserialize(rx_queue_in_.data() + 6 + i * 10, read_frames_[read_frames_count_])) {
                    RTT::Logger::In in("EcCanQueue::readReply");
                    RTT::log(RTT::Error) << "could not deserialize CAN frame: " << RTT::endlog();
                    return false;
                }
                bool match = false;
                if (filters_.size() == 0) {
                    match = true;
                }
                for (int j = 0; j < filters_.size(); ++j) {
                    if ((read_frames_[read_frames_count_].id & filters_[j].second) == (filters_[j].first & filters_[j].second)) {
                        match = true;
                        break;
                    }
                }
                if (match) {
                    ++read_frames_count_;
                    if (read_frames_count_ >= N_FRAMES*5) {
                        break;
                    }
                }
            }
        }

        for (uint16_t i = 0; i < read_frames_count_; ++i) {
            if (read_frames_[i].id == can_id) {
                dlc = read_frames_[i].dlc;
                for (uint16_t j = 0; j < dlc; ++j) {
                    data_out[j] = read_frames_[i].data[j];
                }
                // erase
                for (uint16_t j = i + 1; j < read_frames_count_; ++j) {
                    read_frames_[j-1] = read_frames_[j];
                }
                --read_frames_count_;
                return true;
            }
        }

        return false;
    }

private:
    bool deserialize(const int8_t *ptr, can_frame &frame) {
        const uint8_t *uptr = reinterpret_cast<const uint8_t* >(ptr);
        frame.id = (uptr[0]<<3) | ((uptr[1]>>5)&0x07);
        frame.dlc = (uptr[1]&0x0F);
        if (frame.dlc > 8) {
            return false;
        }
        for (int i = 0; i < frame.dlc; ++i) {
            frame.data[i] = ptr[i + 2];
        }
        return true;
    }

    boost::array<can_frame, N_FRAMES*5 > read_frames_;
    uint16_t read_frames_count_;
    std::vector<std::pair<uint32_t, uint32_t > > filters_;
    RTT::InputPort<boost::array<int8_t, N_FRAMES*10+6 > > port_rx_queue_in_;
    boost::array<int8_t, N_FRAMES*10+6 > rx_queue_in_;

    RTT::OutputPort<can_frame > port_tx_out_;
};

}   // namespace ec_can_queue

}   // namespace controller_common

#endif  // CONTROLLER_COMMON_EC_CAN_QUEUE_H__

