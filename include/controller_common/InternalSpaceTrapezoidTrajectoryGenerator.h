/*
 * Copyright (c) 2010-2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * InternalSpaceSplineTrajectoryGenerator.h
 *
 * Generator for both the motor and joint spline interpolation
 *
 *  Created on: 22-09-2010
 *      Author: Konrad Banachowicz, Dawid Seredynski, Karolina Borkowska
 */

#ifndef CONTROLLER_COMMON_INTERNAL_SPACE_TRAPEZOID_TRAJECTORY_GENERATOR_H_
#define CONTROLLER_COMMON_INTERNAL_SPACE_TRAPEZOID_TRAJECTORY_GENERATOR_H_

#include <Eigen/Dense>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <vector>

#include <iostream>
#include <fstream>
#include <ctime>
#include <stdlib.h> 
#include <cstdlib>

#include "rtt_rosclock/rtt_rosclock.h"

#include "controller_common/velocityprofile_trapezoid.hpp"
#include <trapezoid_trajectory_msgs/TrapezoidTrajectoryAction.h>

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#error EIGEN_RUNTIME_NO_MALLOC must be defined
#endif

template <class TRAJECTORY_TYPE >
class InternalSpaceTrapezoidTrajectoryGenerator : public RTT::TaskContext {
 public:
  explicit InternalSpaceTrapezoidTrajectoryGenerator(const std::string& name);
  virtual ~InternalSpaceTrapezoidTrajectoryGenerator();

  virtual bool configureHook();
  virtual bool startHook();
  virtual void stopHook();
  virtual void updateHook();

 protected:
  typedef Eigen::Matrix<double, TRAJECTORY_TYPE::DOFS, 1>  VectorNd;
  RTT::InputPort<TRAJECTORY_TYPE> port_jnt_command_in_;

  RTT::OutputPort<VectorNd> port_internal_space_position_command_out_;
  RTT::InputPort<VectorNd> port_internal_space_position_measurement_in_;
  RTT::InputPort<bool> port_is_synchronised_in_;

  VectorNd stiffness_command_out_;
  RTT::OutputPort<VectorNd> port_stiffness_command_out_;

  int32_t generator_status_;
  RTT::OutputPort<int32_t> port_generator_status_out_;

 private:
  virtual void saveDataToFile();
  TRAJECTORY_TYPE getNewGoalAndInitData(void);
  void sendPositions();
  bool isAnyJointStillInMotion();
  bool isTheLastTrajectoryPointReached();
  bool setVelocityProfiles(double t);
  void setLastPointsAndSendSuccesMsg();
  void generatePositions(double t);
  void updateHookWithVelocityBasedProfiles(ros::Time now);
  bool phaseTimeHasPassed(double t);
  bool calculatePhaseDuration(double t);
  bool pathToleranceViolated();
  void updateHookWithDurationBasedProfiles(ros::Time now);
  void deactivateTrajectory();
  void prf(double x, std::string name = "[GEN]: ");

  //bool last_point_not_set_;
  bool trajectory_active_;
  std::vector<bool> active_points_; 
  std::vector<KDL::VelocityProfile_Trapezoid> vel_profile_;

  VectorNd des_jnt_pos_, setpoint_, old_point_, first_point_, stiffness_;

  std::vector <VectorNd> setpoint_results_, jnt_results_;

  TRAJECTORY_TYPE trajectory_;
  int trajectory_point_index_;

  ros::Time last_time_;
  int update_hook_iter_;
  std::vector<std::string> jointNames_;

  // properties
  std::vector<double> lowerLimits_;
  std::vector<double> upperLimits_;
  std::vector <double> old_velocities_, end_times_,
                         max_velocities_, max_accelerations_,
                         max_velocities_param_, max_accelerations_param_;

  double phase_end_time_;

  bool research_mode_;
  bool duration_mode_;
  bool save_data_;

  int32_t status;

  std::string errMsg_;
};

using namespace RTT;

template <class TRAJECTORY_TYPE >
InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::InternalSpaceTrapezoidTrajectoryGenerator(
    const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      //last_point_not_set_(false),
      trajectory_active_(false),
      trajectory_point_index_(0),
      port_jnt_command_in_("jnt_INPORT"),
      port_internal_space_position_command_out_("JointPositionCommand_OUTPORT", true),
      port_internal_space_position_measurement_in_("JointPosition_INPORT"),
      port_generator_status_out_("generator_status_OUTPORT"),
      port_is_synchronised_in_("IsSynchronised_INPORT"),
      port_stiffness_command_out_("stiffness_command_OUTPORT"),
      save_data_(false),
      duration_mode_(false),
      research_mode_(true) {
  this->ports()->addPort(port_jnt_command_in_);
  this->ports()->addPort(port_internal_space_position_command_out_);
  this->ports()->addPort(port_internal_space_position_measurement_in_);
  this->ports()->addPort(port_is_synchronised_in_);
  this->ports()->addPort(port_generator_status_out_);
  this->ports()->addPort(port_stiffness_command_out_);

  this->addProperty("lower_limits", lowerLimits_);
  this->addProperty("upper_limits", upperLimits_);
  this->addProperty("joint_names", jointNames_);
  this->addProperty("max_velocities", max_velocities_param_);
  this->addProperty("max_accelerations", max_accelerations_param_);

  return;
}

template <class TRAJECTORY_TYPE >
InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::~InternalSpaceTrapezoidTrajectoryGenerator() {
  return;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::configureHook() {
  RTT::Logger::In in("InternalSpaceTrapezoidTrajectoryGenerator::configureHook");
  setpoint_.resize(TRAJECTORY_TYPE::DOFS);
  first_point_.resize(TRAJECTORY_TYPE::DOFS);
  vel_profile_.resize(TRAJECTORY_TYPE::DOFS);
  active_points_.resize(TRAJECTORY_TYPE::DOFS);

  max_velocities_.resize(TRAJECTORY_TYPE::DOFS);
  max_accelerations_.resize(TRAJECTORY_TYPE::DOFS);

  old_velocities_.resize(TRAJECTORY_TYPE::DOFS);
  end_times_.resize(TRAJECTORY_TYPE::DOFS);
  stiffness_.resize(TRAJECTORY_TYPE::DOFS);

  int data_buffer_size_ = 100000;

  setpoint_results_.resize(data_buffer_size_);
  jnt_results_.resize(data_buffer_size_);

  return true;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::startHook() {
  RESTRICT_ALLOC;
  //std::cout<<max_velocities_[0]<<std::endl;
  //std::cout<<"Starting generator"<<std::endl;

  for(auto& sr: setpoint_results_){
    for(int i=0;i<TRAJECTORY_TYPE::DOFS;++i){
        sr(i)= 0.0;
    }
  }
  for(auto& jr: jnt_results_){
    for(int i=0;i<TRAJECTORY_TYPE::DOFS;++i){
        jr(i)= 0.0;
    }
  }
  //prf(setpoint_(0), "[GEN] initial positions: ");

  if (port_internal_space_position_measurement_in_.read(setpoint_)
      == RTT::NoData) {
    return false;
  }
  first_point_ = setpoint_;
  //port_generator_active_out_.write(true);
  //last_point_not_set_ = false;
  trajectory_active_ = false;

  last_time_ = rtt_rosclock::host_now();
  update_hook_iter_ = 0;

  return true;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::stopHook() {
  
  //port_generator_active_out_.write(false);
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::saveDataToFile(){
  UNRESTRICT_ALLOC;
  time_t now = time(0);
  tm *ltm = localtime(&now);
  const char* home = getenv("HOME");
  std::string path(home);
  char buffer[80];
  strftime(buffer,sizeof(buffer),"%d-%m-%Y--%I:%M:%S",ltm);
  std::string str(buffer);
  path += "/VELMOS_results/trapezoid_generator_results/" + str+"/";
  /*path += "/IRPOS_results/trapezoid_generator_results/"+std::to_string(ltm->tm_mday)+":"+
                                                        std::to_string(ltm->tm_mon)+":"+
                                                        std::to_string(ltm->tm_year)+":"+
                                                        std::to_string(1 + ltm->tm_hour)+":"+
                                                        std::to_string(1 + ltm->tm_min)+":"+
                                                        std::to_string(1 + ltm->tm_sec)+"/";*/
  std::string command = "mkdir -pZ "+path;
  std::system(command.c_str());
  std::ofstream myfile;
  uint finish_line_;
  if(update_hook_iter_>setpoint_results_.size()){
    finish_line_ = setpoint_results_.size();
  } else {
    finish_line_ = update_hook_iter_;
  }

  //std::cout<<setpoint_results_[500].size()<<std::endl;
  myfile.open(path+"setpoints.txt");
  for(int i = 0; i<finish_line_;++i){
    for(int k = 0; k<setpoint_results_[i].size(); ++k){
      //prf( lol, "[GEN][getNewGoalAndInitData] tdcfsdcfsd: " ) ;
      myfile << std::to_string(setpoint_results_[i](k))+ " ";
    }
    myfile << "\n";
  }
  myfile.close();

  myfile.open(path+"results.txt");
  for(int i = 0; i<finish_line_;++i){
    for(int k =0; k<jnt_results_[i].size(); ++k){
      myfile << std::to_string(jnt_results_[i](k))+ " ";
    }
    myfile << "\n";
  }
  myfile.close();
  //save points from msg
  myfile.open(path+"user_setup.txt");
  if(duration_mode_){
    myfile << "duratin mode\n";
  } else {
    myfile << "velocity mode\n";
  }
  myfile << "max velocities\n";
  for(int k =0; k<TRAJECTORY_TYPE::DOFS; ++k){
    myfile << std::to_string(max_velocities_[k])+ " ";
  }
  myfile << "\n";
  myfile << "max accelerations\n";
  for(int k =0; k<TRAJECTORY_TYPE::DOFS; ++k){
    myfile << std::to_string(max_accelerations_[k])+ " ";
  }
  myfile << "\n";
  for(int i = 0; i<trajectory_.count_trj;++i){
    myfile << "starting point\n";
    for(int k =0; k<TRAJECTORY_TYPE::DOFS; ++k){
      myfile << std::to_string(first_point_(k))+ " ";
    }
    myfile << "\n";
    myfile << "setpoint\n";
    for(int k =0; k<TRAJECTORY_TYPE::DOFS; ++k){
      myfile << std::to_string(trajectory_.trj[i].positions[k])+ " ";
    }
    myfile << "\n";
  }
  myfile.close();
  RESTRICT_ALLOC;
}

template <class TRAJECTORY_TYPE >
TRAJECTORY_TYPE InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::getNewGoalAndInitData(){

  TRAJECTORY_TYPE trj_ptr_tmp;
  if (port_jnt_command_in_.read(trj_ptr_tmp) == RTT::NewData) {
    for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
      stiffness_(i) = trj_ptr_tmp.stiffness[i];
    }
    //std::cout<<trajectory_.stiffness[2]<<std::endl;
    if(trj_ptr_tmp.count_trj < 1){
      //prf(trj_ptr_tmp.count_trj, "trj_ptr_tmp.count_trj: ");
      status = trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL;
      //prf(status, "status: ");
      return trj_ptr_tmp;
    }
    //init trajectory data
    trajectory_ = trj_ptr_tmp;
    trajectory_point_index_ = 0;
    //prf(trajectory_point_index_, "[GEN][getNewGoalAndInitData] trajectory_point_index_: ");
    setpoint_= des_jnt_pos_;
    old_point_ = setpoint_;
    first_point_ = setpoint_;
    //check wether research mode is on
    //if so init tabels used in this setting
    research_mode_ = trj_ptr_tmp.research_mode;
    duration_mode_ = trj_ptr_tmp.duration_mode;
    save_data_ = trj_ptr_tmp.save_data;


    for(int i=0; i<trajectory_.count_trj; ++i){
      if(!trj_ptr_tmp.trj[i].use_velocities){
        for(int k=0; k<TRAJECTORY_TYPE::DOFS;++k){
          trajectory_.trj[i].velocities[k]=0.0;
        }
      }
    }

    if(!duration_mode_){
      for(int i=0;i<TRAJECTORY_TYPE::DOFS;i++){
        old_velocities_[i]=0.0;
        end_times_[i]=0.0;
        active_points_[i]=false;
        //std::cout<<"i="<<i<<std::endl;
        //std::cout<<"max_velocities_[i]="<<max_velocities_[i]<<std::endl;
        //std::cout<<"trj_ptr_tmp.max_velocities[i]="<<trj_ptr_tmp.max_velocities[i]<<std::endl;
        max_velocities_[i] = trj_ptr_tmp.max_velocities[i];
        //std::cout<<"foch"<<std::endl;
        max_accelerations_[i] = trj_ptr_tmp.max_accelerations[i];
      }
      //std::cout<<trj_ptr_tmp.max_accelerations[0]<<std::endl;
    } else {
      //has to be reset everytime 
      max_velocities_=max_velocities_param_;
      max_accelerations_=max_accelerations_param_;
      phase_end_time_ = 0.0;
    }
    //init data used for establishing when the movement is over
    //last_point_not_set_ = true;
    trajectory_active_ = true;
    //new_point_ = true;
    //save_data_=true;
    update_hook_iter_=0;
    //std::cout<<"got new trajectory"<<std::endl;
  }
  //if needed a pointer is provided
  return trj_ptr_tmp;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::sendPositions(){

  for (int i = 0; i < TRAJECTORY_TYPE::DOFS; ++i) {
    stiffness_command_out_(i) = stiffness_[i];
    //prf(stiffness_[i], "stiffness_[i]: ");
  }
  port_stiffness_command_out_.write(stiffness_command_out_);
  //std::cout<<status<<std::endl;
  /*if(status == trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL){
    std::cout<<"fuuuuuuuuuuuuck"<<std::endl;
  }*/
  port_generator_status_out_.write(status);
  port_internal_space_position_command_out_.write(setpoint_);

}

template <class TRAJECTORY_TYPE >
inline bool InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::isAnyJointStillInMotion(){
  return std::any_of(active_points_.begin(), active_points_.end(),
                                                  [](bool ap){return ap;});
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::prf(double x, std::string name){
  std::cout<<name<<std::to_string(x)<<std::endl;
}

template <class TRAJECTORY_TYPE >
inline bool InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::isTheLastTrajectoryPointReached(){
  //prf(trajectory_point_index_, "[GEN] trajectory_point_index_: ");
  //prf(trajectory_.points.size(), "[GEN] trajectory_.points.size(): ");
  //std::cout<<std::to_string(trajectory_point_index_>= (int)trajectory_.points.size())<<std::endl;
  return trajectory_point_index_ >= (int)trajectory_.count_trj;
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::setVelocityProfiles(double t){
  int result;
  //std::cout<<"setting profiles"<<std::endl;
  
  for(unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++){
    if(!duration_mode_){
      //std::cout<<"is_velocity_based_"<<std::endl;
      /*prf(t,"t=");
      prf(old_point_(i),"old_point_(i)=");
      prf(trajectory_.points[trajectory_point_index_].positions[i],"trajectory_.points[trajectory_point_index_].positions[i]=");
      prf(old_velocities_[i],"old_velocities_[i]=");
      prf(trajectory_.points[trajectory_point_index_].velocities[i],"trajectory_.points[trajectory_point_index_].velocities[i]=");
      prf(max_velocities_[i],"max_velocities_[i]=");
      prf(max_accelerations_[i],"max_accelerations_[i]=");
      prf(lowerLimits_[i],"lowerLimits_[i]=");
      prf(upperLimits_[i],"upperLimits_[i]=");
      prf(research_mode_,"research_mode_=");*/
      result = vel_profile_[i].SetProfileVelocity(
                                  t,
                                  old_point_(i),
                                  trajectory_.trj[trajectory_point_index_].positions[i],
                                  old_velocities_[i],
                                  trajectory_.trj[trajectory_point_index_].velocities[i],
                                  max_velocities_[i],
                                  max_accelerations_[i],
                                  lowerLimits_[i],
                                  upperLimits_[i],
                                  research_mode_);
      
      if(vel_profile_[i].Duration() != 0.0){
        end_times_[i] = vel_profile_[i].Duration()+t;
        active_points_[i]=true;
        //std::cout<<"set"<<std::endl;
      }
    } else {
      result = vel_profile_[i].SetProfileDuration(
                                  t,
                                  phase_end_time_-t,
                                  old_point_(i),
                                  trajectory_.trj[trajectory_point_index_].positions[i],
                                  old_velocities_[i],
                                  trajectory_.trj[trajectory_point_index_].velocities[i],
                                  max_velocities_[i],
                                  max_accelerations_[i],
                                  lowerLimits_[i],
                                  upperLimits_[i],
                                  research_mode_);
    }
    //vel_profile_[i].printCoeffs();
    if(result != trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL){
      status = result;
      //std::cout<<result<<std::endl;
      //std::cout<<"[GEN]about to sent a failure msg"<<std::endl;
      return false;
    }
  }
  for(unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++){
    old_point_(i) = trajectory_.trj[trajectory_point_index_].positions[i];
    old_velocities_[i] = trajectory_.trj[trajectory_point_index_].velocities[i];
  }
  ++trajectory_point_index_;
  //prf(trajectory_point_index_, "[GEN][setVelocityProfiles] trajectory_point_index_: ");
  return true;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::setLastPointsAndSendSuccesMsg(){
  if(trajectory_.count_trj <= 0){
    return;
  }
  for (unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++) {
    //prf(i, "i: ");
    //prf(trajectory_.count_trj, "trajectory_.count_trj: ");
    setpoint_(i) = trajectory_.trj[trajectory_.count_trj - 1].positions[i];
    active_points_[i]=false;
  }
  status = trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL;
  //std::cout<<setpoint_<<std::endl;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::generatePositions(double t) {
  if(!duration_mode_){
    for(unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++){
      if(end_times_[i]>t && active_points_[i]==true){
        setpoint_(i) = vel_profile_[i].Pos(t);
      } else {
        active_points_[i]=false;
      }
    }
  } else {
    for(unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++){
      setpoint_(i) = vel_profile_[i].Pos(t);
    }
  }

  if(save_data_ && 
     update_hook_iter_ < setpoint_results_.size()){
    jnt_results_[update_hook_iter_] = des_jnt_pos_;
    //std::cout<<"[GEN]pos:"<<setpoint_(0)<<" time:"<<std::to_string(now.toSec())<<std::endl;
    setpoint_results_[update_hook_iter_] = setpoint_;
  }
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::deactivateTrajectory(){
  trajectory_active_=false;
  if(save_data_)
    saveDataToFile();
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::updateHookWithVelocityBasedProfiles(ros::Time now) {

  //std::cout<<"[GEN]trajectory is active"<<std::endl;
  double t = now.toSec();
  if(!isAnyJointStillInMotion()){
    if(isTheLastTrajectoryPointReached()){
      //std::cout<<"[GEN] setting last point"<<std::endl;
      setLastPointsAndSendSuccesMsg();
      deactivateTrajectory();
    } else {
      //std::cout<<"[GEN] setting profiles"<<std::endl;
      bool result = setVelocityProfiles(t);
      if(!result){
        //std::cout<<"[GEN] setting profiles failed"<<std::endl;
        deactivateTrajectory();
      }
      //std::cout<<"[GEN] profiles set"<<std::endl;
    }
  }
  //std::cout<<"[GEN]done the big if"<<std::endl;
  if(trajectory_active_){
    //std::cout<<"[GEN]started generating positions"<<std::endl;
    status = trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::ACTIVE;
    generatePositions(t);
    //std::cout<<"[GEN]ended generating positions"<<std::endl;
  }
}

template <class TRAJECTORY_TYPE >
inline bool InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::phaseTimeHasPassed(double t){
  return (phase_end_time_ <= t);
}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::calculatePhaseDuration(double t){
  double duration_longest = 0.0;
  double duration;
  for(unsigned int i = 0; i < TRAJECTORY_TYPE::DOFS; i++){
    duration = vel_profile_[i].calculateDuration(
                                  old_point_(i),
                                  trajectory_.trj[trajectory_point_index_].positions[i],
                                  old_velocities_[i],
                                  trajectory_.trj[trajectory_point_index_].velocities[i],
                                  max_velocities_[i],
                                  max_accelerations_[i]);
    //std::cout<<"[GEN] duration["<<i<<"]="<<duration<<std::endl;
    if(duration > duration_longest){
      duration_longest = duration;
    }
  }
  //std::cout<<"[GEN] duration_longest="<<duration_longest<<std::endl;
  phase_end_time_ = duration_longest + t;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::updateHookWithDurationBasedProfiles(ros::Time now) {

  double t = now.toSec();
  if(phaseTimeHasPassed(t)){
    //std::cout<<"[GEN] phase time has passed="<<std::endl;
    if(!isTheLastTrajectoryPointReached()){
      //std::cout<<"[GEN] up to set profiles="<<std::endl;
      calculatePhaseDuration(t);
      bool result = setVelocityProfiles(t);
      if(!result){
        deactivateTrajectory();
      }
    } else {
      //std::cout<<"[GEN] sentting last point"<<std::endl;
      setLastPointsAndSendSuccesMsg();
      deactivateTrajectory();
    }
  } 

  if(trajectory_active_){
    //std::cout<<"[GEN] generating positions= "<<trajectory_active_<<std::endl;
    status = trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::ACTIVE;
    generatePositions(t);
  }

}

template <class TRAJECTORY_TYPE >
bool InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE>::pathToleranceViolated(){

  if(trajectory_.count_trj <1){
    return false;
  }
  if(research_mode_){
    return false;
  }
  for (int i = 0; i < trajectory_.path_tolerance.size(); i++) {
    if ( trajectory_.path_tolerance[i] > 0 && fabs(des_jnt_pos_(i)-setpoint_(i)) > trajectory_.path_tolerance[i]){
      /*std::cout<<" trajectory_.path_tolerance[i]=" 
               <<trajectory_.path_tolerance[i]
               <<" des="
               <<des_jnt_pos_(i)
               <<" old="
               <<setpoint_(i)
               <<" fabs="
               <<fabs(des_jnt_pos_(i)-old_point_(i))
               <<std::endl;*/
      RTT::Logger::log(RTT::Logger::Error) << "Path tolerance violated"
            << RTT::endlog();
      return true;
    }
  }
  return false;
}

template <class TRAJECTORY_TYPE >
void InternalSpaceTrapezoidTrajectoryGenerator<TRAJECTORY_TYPE >::updateHook() {
  //std::cout<<"[GEN]in generator"<<std::endl;
  //std::cout<<"zium: "<<TRAJECTORY_TYPE::zium<<std::endl;
  //port_generator_active_out_.write(true);
  //std::cout<<"[GEN]starting getNewGoalAndInitData"<<std::endl;
  port_internal_space_position_measurement_in_.read(des_jnt_pos_);
  getNewGoalAndInitData();
  //std::cout<<"[GEN]starting adjustTimeFrames"<<std::endl;
  ros::Time now = rtt_rosclock::host_now();

  if(trajectory_active_){
    if(pathToleranceViolated()){
      status = trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      deactivateTrajectory();
    }
    if(!duration_mode_){
      //std::cout<<"[GEN]starting updateHookWithVelocityBasedProfiles"<<std::endl;
      updateHookWithVelocityBasedProfiles(now);
      //std::cout<<"[GEN]ended updateHookWithVelocityBasedProfiles"<<std::endl;
    } else {
      //std::cout<<"[GEN]starting updateHookWithDurationBasedProfiles"<<std::endl;
      updateHookWithDurationBasedProfiles(now);
      //std::cout<<"[GEN]ended updateHookWithDurationBasedProfiles"<<std::endl;
    }

  }
  sendPositions();
  //std::cout<<"[GEN]started sending positions"<<std::endl;
  //std::cout<<"[GEN]ended sending positions"<<std::endl;

  update_hook_iter_++;

}




#endif  // CONTROLLER_COMMON_INTERNAL_SPACE_TRAPEZOID_TRAJECTORY_GENERATOR_H_

