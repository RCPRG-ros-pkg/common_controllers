/*
 * Copyright (c) 2010-2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
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

#include <limits>
#include <assert.h>
#include <math.h> 

#include "controller_common/velocityprofile_trapezoid.hpp"

#include <trapezoid_trajectory_msgs/TrapezoidTrajectoryResult.h>


namespace KDL {

static inline void generatePowers(int n, double x, double* powers) {
  powers[0] = 1.0;
  for (int i = 1; i <= n; i++) {
    powers[i] = powers[i - 1] * x;
  }
  return;
}


VelocityProfile_Trapezoid::VelocityProfile_Trapezoid() {
  fillCoeffsWithZeros();
  duration_ = 0.0;
  //error_msg_ = " ";
  epsi_ = 0.01;
  return;
}

VelocityProfile_Trapezoid::VelocityProfile_Trapezoid(
    const VelocityProfile_Trapezoid &p) {
  duration_ = p.duration_;
  ap_time_ = p.ap_time_;
  dp_time_ = p.dp_time_;
  bg_time_ = p.bg_time_;

  ap_coeff_[0] = p.ap_coeff_[0];
  ap_coeff_[1] = p.ap_coeff_[1];
  ap_coeff_[2] = p.ap_coeff_[2];

  cvp_coeff_[0] = p.cvp_coeff_[0];
  cvp_coeff_[1] = p.cvp_coeff_[1];
  cvp_coeff_[2] = p.cvp_coeff_[2];

  dp_coeff_[0] = p.dp_coeff_[0];
  dp_coeff_[1] = p.dp_coeff_[1];
  dp_coeff_[2] = p.dp_coeff_[2];

  //error_msg_ = p.error_msg_;

  return;
}

void VelocityProfile_Trapezoid::fillCoeffsWithZeros(){
  ap_time_ = 0.0;
  dp_time_ = 0.0;
  bg_time_ = 0.0;

  ap_coeff_[0] = 0.0;
  ap_coeff_[1] = 0.0;
  ap_coeff_[2] = 0.0;

  cvp_coeff_[0] = 0.0;
  cvp_coeff_[1] = 0.0;
  cvp_coeff_[2] = 0.0;

  dp_coeff_[0] = 0.0;
  dp_coeff_[1] = 0.0;
  dp_coeff_[2] = 0.0;

}

//std::string VelocityProfile_Trapezoid::getErrorMsg() const{
//  return error_msg_;
//}

//void VelocityProfile_Trapezoid::setLimitValues(double vel_max, double acc_max, double pos_min, double pos_max){
//  vel_max_ = vel_max;
//  acc_max_ = acc_max;
//  pos_min_ = pos_min;
//  pos_max_ = pos_max;
//}

bool VelocityProfile_Trapezoid::trajectoryIsFeasible(
      double distance, int sign, double vel1, double vel2,
      double vel_max, double acc_max, bool research_mode, bool duration){

  double vel_diff = vel1*vel1-vel2*vel2;
  bool error = false;
  if(vel_diff < 0.0){
    vel_diff = (-1)*vel_diff;
  }

  // max_vel that is zero makes it impossible to calculate duration 
  // only possible when displacemat is zero, but it is assumed that this function
  // shuold not be used then (no displacemet -> no trajectory to speak of)
  if (vel_max*sign <= 0.0 
      && !duration){
    error = true;
    //error_msg_ = "Max velocity is non-positive. vel_max="+std::to_string(vel_max)+";";
  } else if(vel_max*sign < 0.0){
    error = true;
    //error_msg_ = "Max velocity is less than zero. vel_max="+std::to_string(vel_max)+";";
  }
  // check wether boundary velocities are greater then max velocity
  // needed in research mode wich assumes max velocity is a physical
  // limit rather then velocity to be established in constant phase
  else if((vel1 != 0.0 ||
           vel2 != 0.0) && 
           research_mode){
    error = true;
    //error_msg_ = "Boundary velocity greater then zero in research mode. vel1="+std::to_string(vel1)+
    //             "; vel2="+std::to_string(vel2)+"; research_mode="+std::to_string(research_mode)+";";
  }
  // check wether it is possible to achieve boundary velocities
  // when one of them is greater then max_velocity 
  // (here treated as const velocit rather then reachable limit, hence
  // in research mode even those cases won't be allowed)
  //else if((sign*vel1 - vel_max > 0) || 
  //        (sign*vel2 - vel_max > 0)){
  //  error_msg_ = "Boundary velocity error. vel1="+std::to_string(vel1)+
  //               "; vel2="+std::to_string(vel2)+"; vel_max="+std::to_string(vel_max)+";";
  //}
  // check wether displacement is too small for given acceleration
  // page 71 in Trajectory Planning for Automatic Machines and Robots
  // by Luigi Biagiotti & Claudio Melchiorri 
  else if(acc_max*distance < (vel_diff/2)){
    error = true;
    //error_msg_ = "Displacement is too small for given acceleration. vel1=" +std::to_string(vel1)+
    //             "; vel2="+std::to_string(vel2)+"; acc_max="+std::to_string(acc_max)+
     //            "; displacement="+std::to_string(distance)+ ";";
  }
  return !error;
}

bool VelocityProfile_Trapezoid::calculateCoeffs(
  double pos1, double pos2, double vel1, double vel2, double vel_max){
  //fillCoeffsWithZeros();
  if(duration_<= 0.0){
    //error_msg_ = "Duration less, or equal zero with nonzero distance. duration_="+std::to_string(duration_)+
    //             "; displacement="+std::to_string((pos2-pos1))+";";
    return false;
  }
  if(ap_time_>0){
    ap_coeff_[0] = pos1 - vel1*bg_time_ + (vel_max-vel1)*(bg_time_*bg_time_)/(2*ap_time_);
    ap_coeff_[1] = vel1 - (vel_max-vel1)*bg_time_/ap_time_;
    ap_coeff_[2] = (vel_max-vel1)/(2*ap_time_);
  }
  cvp_coeff_[0] = pos1 + (vel1-vel_max)*ap_time_/2 - vel_max*bg_time_;
  cvp_coeff_[1] = vel_max;
  cvp_coeff_[2] = 0.0;
  if(dp_time_>0){
    dp_coeff_[0] = pos2 - vel2*(bg_time_+duration_) - (vel_max-vel2)*((bg_time_+duration_)*(bg_time_+duration_))/(2*dp_time_);
    dp_coeff_[1] = vel2 + (vel_max-vel2)*(bg_time_+duration_)/dp_time_;
    dp_coeff_[2] = (vel2-vel_max)/(2*dp_time_);
  }
  //printCoeffs();
  //prf(bg_time_, "begin time: ");
  //prf(pos1, "pos1: ");
  //prf(pos2, "pos2: ");
  //prf(vel1, "vel1: ");
  //prf(vel2, "vel2: ");
  //prf(vel_max, "vel_max: ");
  //prf(ap_time_, "ap_time_: ");
  //prf(dp_time_, "dp_time_: ");
  return true;
}

bool VelocityProfile_Trapezoid::durationIsTooShort(){
  if (ap_time_ == 0.0){
    return false;
  }
  double time = ap_time_ + bg_time_;
  double pos_v = cvp_coeff_[0] + time*cvp_coeff_[1] + time*time*cvp_coeff_[2];
  double pos_a = ap_coeff_[0] + time*ap_coeff_[1] + time*time*ap_coeff_[2];
  if( (pos_v < pos_a - epsi_ ||
       pos_v > pos_a + epsi_) &&
       time){
    //error_msg_ = "Duration is too short. pos_v=" +std::to_string(pos_v)+ "; pos_a=" +std::to_string(pos_a)+ ";";
    return true;
  } else {
    return false;
  }
}

bool VelocityProfile_Trapezoid::durationIsTooLong(){
  if(dp_time_ == 0.0){
    return false;
  }
  double time = duration_ + bg_time_ - dp_time_;
  double pos_v = cvp_coeff_[0] + time*cvp_coeff_[1] + time*time*cvp_coeff_[2];
  double pos_d = dp_coeff_[0] + time*dp_coeff_[1] + time*time*dp_coeff_[2];
  if( pos_v < pos_d - epsi_ ||
      pos_v > pos_d + epsi_){
    //error_msg_ = "Duration is too long. pos_v=" +std::to_string(pos_v)+ "; pos_d=" +std::to_string(pos_d)+
    //                       "; duration=" +std::to_string(duration_)+ ";";
    return true;
  } else {
    return false;
  }
}

void VelocityProfile_Trapezoid::printCoeffs(){
  prf(ap_coeff_[0]);
  prf(ap_coeff_[1]);
  prf(ap_coeff_[2]);
  prf(cvp_coeff_[0]);
  prf(cvp_coeff_[1]);
  prf(cvp_coeff_[2]);
  prf(dp_coeff_[0]);
  prf(dp_coeff_[1]);
  prf(dp_coeff_[2]);
}

void VelocityProfile_Trapezoid::prf(double x, std::string name){
}

bool VelocityProfile_Trapezoid::maxVelocityIsUnreachable(
      double distance, double vel1, double vel2, double vel_max, double acc_max, int sign){
  if(distance*acc_max < (vel_max*vel_max - (vel1*vel1 + vel2*vel2)/2)){
    return true;
  }
  return false;
}

bool VelocityProfile_Trapezoid::accMaxIsEnoughForDuration(
      double distance, double vel1, double vel2, double acc_max){
  double limit = epsi_ - (2*distance 
                 - duration_*(vel1+vel2) 
                 + sqrt(4*distance*distance 
                        - 4*distance*(vel1+vel2)*duration_ 
                        + 2*(vel1*vel1+vel2*vel2)*duration_*duration_
                        )
                 )/(duration_*duration_);
  if(acc_max < limit){
    //error_msg_ = "acceleration is too small. acc_max="+std::to_string(acc_max)
    //                + "; limit_="+std::to_string(limit)
    //                + "; duration_="+std::to_string(duration_)
    //                + "; distance="+std::to_string(distance)
     //               + "; vel1="+std::to_string(vel1)+ "; vel2="+std::to_string(vel2);
    return false;
  }
  return true;
}

bool VelocityProfile_Trapezoid::trajectoryBreachesPositionLimits(double pos_min, double pos_max){
    double time_a = (-1)*ap_coeff_[1]/(2*ap_coeff_[2]);
    double time_d = (-1)*dp_coeff_[1]/(2*dp_coeff_[2]);
    bool error = false;
    if(bg_time_ < time_a+bg_time_ && 
       time_a+bg_time_ < bg_time_+ap_time_ &&
       (Pos(time_a)>pos_max || Pos(time_a)<pos_min)){
      error = true;
      //error_msg_ = "Position breached further into trajectory. time_a=" +std::to_string(time_a)+ ";";
    }
    if(bg_time_+duration_-dp_time_ < time_d+bg_time_ && 
       time_d+bg_time_ < bg_time_+duration_ &&
       (Pos(time_d)>pos_max || Pos(time_d)<pos_min)){
      error = true;
    }
    return error;
  }

VelocityProfile_Trapezoid::~VelocityProfile_Trapezoid() {
  return;
}

void VelocityProfile_Trapezoid::SetProfile(double pos1, double pos2) {
  assert(("Function SetProfile is unusable when dealing"
           "with trapezoidal velocity profile.", 0));
  return;
}

void VelocityProfile_Trapezoid::SetProfileDuration(double pos1, double pos2,
                                                         double duration){
  assert(("Function SetProfileDuration is unusable due to lack of values"
           "for returning", 0));
  return;
}

int VelocityProfile_Trapezoid::SetProfileDuration(double duration,
                                                     double pos1, double pos2,
                                                     double vel_max, double acc_max,
                                                     double pos_min, double pos_max,
                                                     bool research_mode){
  return SetProfileDuration(0.0, duration, pos1, pos2, 
                                0.0, 0.0, vel_max, acc_max, 
                                pos_min, pos_max, research_mode);
}

int VelocityProfile_Trapezoid::SetProfileDuration(double duration,
                                                     double pos1, double pos2,
                                                     double vel1, double vel2,
                                                     double vel_max, double acc_max,
                                                     double pos_min, double pos_max,
                                                     bool research_mode){
  return SetProfileDuration(0.0, duration, pos1, pos2, 
                                vel1, vel2, vel_max, acc_max, 
                                pos_min, pos_max, research_mode);
}

int VelocityProfile_Trapezoid::SetProfileDuration(double time1, double duration,
                                                     double pos1, double pos2,
                                                     double vel1, double vel2,
                                                     double vel_max, double acc_max,
                                                     double pos_min, double pos_max,
                                                     bool research_mode){
  //error_msg_ = " ";
  //fillCoeffsWithZeros();
  //duration_=0.0;
  duration_ = duration;

  if(duration_==0.0){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL;
  }

  double distance = pos2-pos1;
  int sign = 1;
  if(distance < 0.0){
    sign = -1;
  }

  bg_time_ = time1;
  distance = distance*sign;
  pos1 = pos1*sign;
  pos2 = pos2*sign;
  vel1 = vel1*sign;
  vel2 = vel2*sign;

  if(!accMaxIsEnoughForDuration(distance, vel1, vel2, acc_max)){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::ACC_TOO_SMALL_FOR_DURATION;
  }

  double tmp = (acc_max*acc_max)*(duration_*duration_)
                          -4*acc_max*distance+2*acc_max*(vel1+vel2)*duration_-(vel1-vel2)*(vel1-vel2);
  if (tmp < 0.0  && tmp+epsi_ > 0.0){
    tmp = 0.0;
  }
  double vel = 0.5*(vel1+vel2+acc_max*duration_-sqrt(tmp));

  if(vel > vel_max + epsi_ || vel < -1*vel_max - epsi_){
    //error_msg_ = "impossible velocity. vel=" +std::to_string(vel)
    //                + "; vel_max="+std::to_string(vel_max)
    //                + "; acc_max="+std::to_string(acc_max)
    //                + "; duration_="+std::to_string(duration_)
    //                + "; distance="+std::to_string(distance)
     //               + "; vel1="+std::to_string(vel1)+ "; vel2="+std::to_string(vel2);
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::IMPOSSIBLE_VELOCITY;
  }

  pos1 = pos1*sign;
  pos2 = pos2*sign;
  vel1 = vel1*sign;
  vel2 = vel2*sign;
  acc_max = acc_max*sign;
  vel_max = sign*vel;
  distance = distance*sign;

  ap_time_ = calculatePhaseTime(vel_max,vel1,acc_max);
  dp_time_ = calculatePhaseTime(vel_max,vel2,acc_max);

  if(!trajectoryIsFeasible(distance, sign, vel1, vel2,vel_max,acc_max, research_mode, true)){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::TRAJECTORY_NOT_FEASIBLE;
  }

  /*prf(duration_,"duration_: ");
  prf(ap_time_,"ap_time_: ");
  prf(dp_time_,"dp_time_: ");
  prf(vel, "vel: ");
  prf(distance,"distance: ");
  prf(acc_max,"acc_max: ");
  prf(bg_time_,"bg_time_: ");*/
  if(!calculateCoeffs(pos1, pos2, vel1, vel2, vel_max)){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::CANT_CALCULATE_COEFFS;
  }

  if(durationIsTooShort()){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::TRAJECTORY_JUMP_ACV;
  } else if (durationIsTooLong()){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::TRAJECTORY_JUMP_CVD;
  }

  if(trajectoryBreachesPositionLimits(pos_min, pos_max)){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::BREACHED_POS_LIMIT;
  }

  return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL;
}

bool VelocityProfile_Trapezoid::isJointAlreadyInPlace(double distance){
  if(distance < epsi_ && distance > -epsi_){
    return true;
  }
  return false;
}

double VelocityProfile_Trapezoid::calculateDuration(double pos1, double pos2,
                                                    double vel1, double vel2,
                                                    double vel_max, double acc_max){
  double distance = pos2-pos1;
  int sign = 1;
  if(distance < 0.0){
    sign = -1;
  }
 
  acc_max = acc_max*sign;
  vel_max = vel_max*sign;
  /*prf(distance, "distance= ");
  prf(vel_max, "vel_max= ");
  prf(acc_max, "acc_max= ");*/

  duration_ = distance/vel_max
              + (vel_max/(2*acc_max))*(((1- vel1/vel_max)*(1- vel1/vel_max))
              + ((1- vel2/vel_max)*(1- vel2/vel_max)));

  // check if, despite possibility of reching the goal, it will be done with smaller velocity
  if(maxVelocityIsUnreachable(distance, vel1, vel2, vel_max, acc_max, sign)){
    vel_max=sign*sqrt(distance*acc_max + (vel1*vel1 + vel2*vel2)/2);
    duration_=calculatePhaseTime(vel_max,vel1,acc_max)
              +calculatePhaseTime(vel_max,vel2,acc_max);
  }
  return duration_;
}


int VelocityProfile_Trapezoid::SetProfileVelocity(double pos1, double pos2,
                                                  double vel_max, double acc_max,
                                                  double pos_min, double pos_max,
                                                  bool research_mode){
  return SetProfileVelocity(0.0, pos1, pos2, 0.0, 0.0, vel_max, acc_max, pos_min, pos_max);
}



int VelocityProfile_Trapezoid::SetProfileVelocity(double pos1, double pos2, 
                                                  double vel1, double vel2, 
                                                  double vel_max, double acc_max, 
                                                  double pos_min, double pos_max,
                                                  bool research_mode){
  return SetProfileVelocity(0.0, pos1, pos2, vel1, vel2, vel_max, acc_max, pos_min, pos_max);
}


int VelocityProfile_Trapezoid::SetProfileVelocity(double time1,
                                                  double pos1, double pos2,
                                                  double vel1, double vel2,
                                                  double vel_max, double acc_max,
                                                  double pos_min, double pos_max,
                                                  bool research_mode){
  //error_msg_ = " ";
  duration_ = 0.0;
  //fillCoeffsWithZeros();
  //duration_=0.0;
  double distance = pos2-pos1;
  int sign = 1;
  if(distance < 0.0){
    sign = -1;
  }

  bg_time_ = time1;
  acc_max = acc_max*sign;
  vel_max = vel_max*sign;

  if(isJointAlreadyInPlace(distance)){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL;
  }

  if(!trajectoryIsFeasible(distance, sign, vel1, vel2,vel_max,acc_max, research_mode)){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::TRAJECTORY_NOT_FEASIBLE;
  }

  //calculate duration of the movement
  duration_ = distance/vel_max
              + (vel_max/(2*acc_max))*(((1- vel1/vel_max)*(1- vel1/vel_max))
              + ((1- vel2/vel_max)*(1- vel2/vel_max)));
  
  ap_time_ = calculatePhaseTime(vel_max,vel1,acc_max);
  dp_time_ = calculatePhaseTime(vel_max,vel2,acc_max);

  // check if, despite possibility of reching the goal, it will be done with smaller velocity
  if(maxVelocityIsUnreachable(distance, vel1, vel2, vel_max, acc_max, sign)){
    if(research_mode){
      return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::MAX_VEL_UNREACHEABLE;
    } else {
      vel_max=sign*sqrt(distance*acc_max + (vel1*vel1 + vel2*vel2)/2);
      ap_time_ = calculatePhaseTime(vel_max,vel1,acc_max);
      dp_time_ = calculatePhaseTime(vel_max,vel2,acc_max);
      duration_=ap_time_+dp_time_;
    }
  }
  //prf(duration_,"duration_:");
  if(!calculateCoeffs(pos1, pos2, vel1, vel2, vel_max)){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::CANT_CALCULATE_COEFFS;
  }

  if(durationIsTooShort()){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::TRAJECTORY_JUMP_ACV;
  } else if (durationIsTooLong()){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::TRAJECTORY_JUMP_CVD;
  }

  if(trajectoryBreachesPositionLimits(pos_min, pos_max)){
    return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::BREACHED_POS_LIMIT;
  }

  return trapezoid_trajectory_msgs::TrapezoidTrajectoryResult::SUCCESSFUL;

}

double VelocityProfile_Trapezoid::calculatePhaseTime(double vel_max, double vel, double acc_max){
  double ptime = (vel_max-vel)/acc_max;
  if(ptime<0.0){
    ptime = (-1.0)*ptime;
  }
  return ptime;
}

double VelocityProfile_Trapezoid::Duration() const {
  return duration_;
}


double VelocityProfile_Trapezoid::Pos(double time) const {
  double t[3];
  double position;
  generatePowers(2, time, t);

  if(time < bg_time_+ap_time_){
    position = t[0] * ap_coeff_[0] + t[1] * ap_coeff_[1] + t[2] * ap_coeff_[2];
  }else if(time < (bg_time_+duration_-dp_time_)){
    position = t[0] * cvp_coeff_[0] + t[1] * cvp_coeff_[1] + t[2] * cvp_coeff_[2];
  }else{
    position = t[0] * dp_coeff_[0] + t[1] * dp_coeff_[1] + t[2] * dp_coeff_[2];
  }

  return position;
}

double VelocityProfile_Trapezoid::Vel(double time) const {
  double t[2];
  double velocity;
  generatePowers(1, time, t);

  if(time < bg_time_+ap_time_){ 
    velocity = t[0] * ap_coeff_[1] + 2.0 * t[1] * ap_coeff_[2];
  }else if(time < (bg_time_+duration_-dp_time_)){
    velocity = t[0] * cvp_coeff_[1] + 2.0 * t[1] * cvp_coeff_[2];
  }else{
    velocity = t[0] * dp_coeff_[1] + 2.0 * t[1] * dp_coeff_[2];
  }

  return velocity;
}

double VelocityProfile_Trapezoid::Acc(double time) const {
  double t[1];
  double acceleration;
  generatePowers(0, time, t);

  if(time < bg_time_+ap_time_){
    acceleration = 2.0 * t[0] * ap_coeff_[2];
  }else if(time < (bg_time_+duration_-dp_time_)){ 
    acceleration = 2.0 * t[0] * cvp_coeff_[2];
  }else{
    acceleration = 2.0 * t[0] * dp_coeff_[2];
  }

  return acceleration;
}

void VelocityProfile_Trapezoid::Write(std::ostream& os) const {
  os << "coefficients : [ " << ap_coeff_[0] << " " << ap_coeff_[1] << " " << ap_coeff_[2]
      << " " << cvp_coeff_[0] << " " << cvp_coeff_[1]<< " " << cvp_coeff_[2]
      << " " << dp_coeff_[0] << " " << dp_coeff_[1]<< " " << dp_coeff_[2] << " ]";
  return;
}

VelocityProfile* VelocityProfile_Trapezoid::Clone() const {
  return new VelocityProfile_Trapezoid(*this);
}

}  // namespace KDL
