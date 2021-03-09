/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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

#include <vector>
#include <string>
#include <controller_common/elmo_servo_state.h>

namespace controller_common {

namespace elmo_servo {

const uint16_t SW_ReadyToSwitchOn_Mask = 0x0001;        // 0000.0000.0001
const uint16_t SW_SwitchedOn_Mask = 0x0002;             // 0000.0000.0010
const uint16_t SW_OperationEnabled_Mask = 0x0004;       // 0000.0000.0100
const uint16_t SW_Fault_Mask = 0x0008;                  // 0000.0000.1000
const uint16_t SW_VoltageEnabled_Mask = 0x0010;         // 0000.0001.0000
const uint16_t SW_QuickStop_Mask = 0x0020;              // 0000.0010.0000
const uint16_t SW_SwitchOnDisabled_Mask = 0x0040;       // 0000.0100.0000
const uint16_t SW_Warning_Mask = 0x0080;                // 0000.1000.0000
const uint16_t SW_ManufactureSpecific_Mask = 0x0100;    // 0001.0000.0000
const uint16_t SW_Remote_Mask = 0x0200;                 // 0010.0000.0000
const uint16_t SW_TargetReached_Mask = 0x0400;          // 0100.0000.0000
const uint16_t SW_InternalLimitActive_Mask = 0x0800;    // 1000.0000.0000
const uint16_t SW_HomingComplete_Mask = 0x1000;         //

const uint16_t NotReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_SwitchOnDisabled_Mask;
const uint16_t NotReadyToSwitchOn_Pattern = 0x0000;

const uint16_t SwitchOnDisabled_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_SwitchOnDisabled_Mask;
const uint16_t SwitchOnDisabled_Pattern = SW_SwitchOnDisabled_Mask;

const uint16_t ReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t ReadyToSwitchOn_Pattern = SW_ReadyToSwitchOn_Mask
    | SW_QuickStop_Mask;

const uint16_t SwitchedOn_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask
    | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_QuickStop_Mask
    | SW_SwitchOnDisabled_Mask;
const uint16_t SwitchedOn_Pattern = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask
    | SW_QuickStop_Mask;

const uint16_t OperationEnabled_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t OperationEnabled_Pattern = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_QuickStop_Mask;

const uint16_t Fault_Reaction_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_SwitchOnDisabled_Mask;
const uint16_t Fault_Reaction_Pattern = SW_Fault_Mask | SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask;

const uint16_t Fault_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask
    | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t Fault_Pattern = SW_Fault_Mask;

const uint16_t QuickStopActive_Mask = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask
    | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16_t QuickStopActive_Pattern = SW_ReadyToSwitchOn_Mask
    | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask;

const std::string servoStateNames[9] = {
    "INVALID",
    "NOT_READY_TO_SWITCH_ON",
    "SWITCH_ON_DISABLED",
    "READY_TO_SWITCH_ON",
    "SWITCH_ON",
    "OPERATION_ENABLED",
    "QUICK_STOP_ACTIVE",
    "FAULT_REACTION_ACTIVE",
    "FAULT",
};

ServoState getServoState(uint16_t statusword) {
  if ((statusword & NotReadyToSwitchOn_Mask) == NotReadyToSwitchOn_Pattern) {
    return ServoState::NOT_READY_TO_SWITCH_ON;
  } else if ((statusword & SwitchOnDisabled_Mask) == SwitchOnDisabled_Pattern) {
    return ServoState::SWITCH_ON_DISABLED;
  } else if ((statusword & ReadyToSwitchOn_Mask) == ReadyToSwitchOn_Pattern) {
    return ServoState::READY_TO_SWITCH_ON;
  } else if ((statusword & SwitchedOn_Mask) == SwitchedOn_Pattern) {
    return ServoState::SWITCH_ON;
  } else if ((statusword & OperationEnabled_Mask) == OperationEnabled_Pattern) {
    return ServoState::OPERATION_ENABLED;
  } else if ((statusword & QuickStopActive_Mask) == QuickStopActive_Pattern) {
    return ServoState::QUICK_STOP_ACTIVE;
  } else if ((statusword & Fault_Reaction_Mask) == Fault_Reaction_Pattern) {
    return ServoState::FAULT_REACTION_ACTIVE;
  } else if ((statusword & Fault_Mask) == Fault_Pattern) {
    return ServoState::FAULT;
  } else {
    return ServoState::INVALID;
  }
}

uint16_t getStatusWord(ServoState s) {
    switch (s) {
    case ServoState::NOT_READY_TO_SWITCH_ON:
        return NotReadyToSwitchOn_Mask & NotReadyToSwitchOn_Pattern;
    case ServoState::SWITCH_ON_DISABLED:
        return SwitchOnDisabled_Mask & SwitchOnDisabled_Pattern;
    case ServoState::READY_TO_SWITCH_ON:
        return ReadyToSwitchOn_Mask & ReadyToSwitchOn_Pattern;
    case ServoState::SWITCH_ON:
        return SwitchedOn_Mask & SwitchedOn_Pattern;
    case ServoState::OPERATION_ENABLED:
        return OperationEnabled_Mask & OperationEnabled_Pattern;
    case ServoState::QUICK_STOP_ACTIVE:
        return QuickStopActive_Mask & QuickStopActive_Pattern;
    case ServoState::FAULT_REACTION_ACTIVE:
        return Fault_Reaction_Mask & Fault_Reaction_Pattern;
    case ServoState::FAULT:
        return Fault_Mask & Fault_Pattern;
    }
    return 0xBAD;   // ServoState::INVALID
}

const std::string& getServoStateStr(ServoState s) {
    switch (s) {
    case ServoState::INVALID:
        return servoStateNames[0];
    case ServoState::NOT_READY_TO_SWITCH_ON:
        return servoStateNames[1];
    case ServoState::SWITCH_ON_DISABLED:
        return servoStateNames[2];
    case ServoState::READY_TO_SWITCH_ON:
        return servoStateNames[3];
    case ServoState::SWITCH_ON:
        return servoStateNames[4];
    case ServoState::OPERATION_ENABLED:
        return servoStateNames[5];
    case ServoState::QUICK_STOP_ACTIVE:
        return servoStateNames[6];
    case ServoState::FAULT_REACTION_ACTIVE:
        return servoStateNames[7];
    case ServoState::FAULT:
        return servoStateNames[8];
    }
    return servoStateNames[0];
}

}   // namespace elmo_servo

}   // namespace controller_common

