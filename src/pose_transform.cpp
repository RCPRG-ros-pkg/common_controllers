// Copyright 2014 WUT
/*
 * pose_transform.cpp
 *
 *  Created on: 11 apr 2014
 *      Author: mwalecki
 */

#include "pose_transform.h"

#include <string>

PoseTransform::PoseTransform(const std::string &name) :
    RTT::TaskContext(name, PreOperational),
    port_target_pass_through_pose_("PrimaryTargetPassThrough_INPORT"),
    port_primary_target_pose_("PrimaryTargetPoint_INPORT"),
    port_primary_frame_selector_("PrimaryFrameSelector_INPORT"),
    port_secondary_target_pose_("SecondaryTargetPoint_OUTPORT", false) {

  this->ports()->addPort(port_target_pass_through_pose_);
  this->ports()->addPort(port_primary_target_pose_);
  this->ports()->addPort(port_primary_frame_selector_);
  this->ports()->addPort(port_secondary_target_pose_);

  port_secondary_target_pose_.setDataSample(secondary_target_pose_);

  this->addProperty("input_frames", input_frames_);
  // variable init
  primary_frame_selector = 0;
  primary_target_status = pose_none;
  target_pass_through_status = pose_none;
}

PoseTransform::~PoseTransform() {
}

bool PoseTransform::configureHook() {
  port_primary_frame_pose_.resize(input_frames_ + 1);
  primary_frame_pose_.resize(input_frames_ + 1);
  primary_frame.resize(input_frames_ + 1);
  primary_frame_status.resize(input_frames_ + 1);

  for (size_t i = 1; i <= input_frames_; i++) {
    // port "0" will not be initialized - "0" is base frame
    char port_name[16];
    snprintf(port_name, sizeof(port_name), "PrimaryFrame%zu", i);  // "PrimaryFrame0" is base frame
    port_primary_frame_pose_[i] = new RTT::InputPort<geometry_msgs::Pose>();
    this->ports()->addPort(port_name, *port_primary_frame_pose_[i]);
  }

  for (size_t i = 0; i <= input_frames_; i++) {
    primary_frame_status[i] = pose_none;
  }
  primary_target_status = pose_none;
  target_pass_through_status = pose_none;

  // Initialize base frame
  primary_frame_pose_[0].position.x = 0;
  primary_frame_pose_[0].position.y = 0;
  primary_frame_pose_[0].position.z = 0;
  primary_frame_pose_[0].orientation.x = 0;
  primary_frame_pose_[0].orientation.y = 0;
  primary_frame_pose_[0].orientation.z = 0;
  primary_frame_pose_[0].orientation.w = 1;
  primary_frame_status[0] = pose_old;
  // Default primary frame is base
  primary_frame_selector = 0;
  return true;
}

void PoseTransform::updateHook() {
  geometry_msgs::Pose pos;

  if (port_primary_frame_selector_.read(primary_frame_selector)
      == RTT::NewData) {
  }

  if (primary_frame_selector < 0) {
    // Say what?
  } else if (primary_frame_selector == 0) {
    primary_frame_status[0] = pose_new;  // Base frame to base frame transform
  } else if (primary_frame_selector <= input_frames_) {
    if (port_primary_frame_pose_[primary_frame_selector]->read(
        primary_frame_pose_[primary_frame_selector]) == RTT::NewData) {
      primary_frame_status[primary_frame_selector] = pose_new;
      tf::poseMsgToKDL(primary_frame_pose_[primary_frame_selector],
                       primary_frame[primary_frame_selector]);
    }
  } else {
    target_pass_through_status = pose_new;
  }

  if (port_primary_target_pose_.read(primary_target_pose_) == RTT::NewData) {
    primary_target_status = pose_new;
    tf::poseMsgToKDL(primary_target_pose_, primary_target);
  }

  if (port_target_pass_through_pose_.read(target_pass_through_pose_) == RTT::NewData) {
    target_pass_through_status = pose_new;
  }

  // On any change of primary_frame or primary_target or target_pass_through
  if ((primary_frame_status[primary_frame_selector] + primary_target_status
       + target_pass_through_status) >= (pose_new + pose_old + pose_old)) {
    primary_frame_status[primary_frame_selector] = pose_old;
    primary_target_status = pose_old;
    target_pass_through_status = pose_old;

    // If you have a Frame F_A_B that expresses the pose of frame B wrt frame A,
    // and a Frame F_B_C that expresses the pose of frame C wrt to frame B,
    // the calculation of Frame F_A_C that expresses the pose of frame C wrt to frame A is as follows:
    if (primary_frame_selector <= (input_frames_)) {
      // Frame F_A_C = F_A_B * F_B_C;
      secondary_target = primary_frame[primary_frame_selector] * primary_target;

      tf::poseKDLToMsg(secondary_target, secondary_target_pose_);
    } else {
      secondary_target_pose_ = target_pass_through_pose_;
    }
    port_secondary_target_pose_.write(secondary_target_pose_);
  }
}
