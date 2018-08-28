#ifndef VELOCITYPROFILE_TRAPEZOID_H
#define VELOCITYPROFILE_TRAPEZOID_H

#include "kdl/velocityprofile.hpp"

#include <string.h>

namespace KDL
{
    /**
     * \brief A trapezoid VelocityProfile trajectory interpolation.
     * @ingroup Motion
     */
class VelocityProfile_Trapezoid : public VelocityProfile
{
public:
    VelocityProfile_Trapezoid();
    VelocityProfile_Trapezoid(const VelocityProfile_Trapezoid &p);

    virtual ~VelocityProfile_Trapezoid();
    
    virtual void SetProfile(double pos1, double pos2);

    virtual void SetProfileDuration(
      double pos1, double pos2, double duration);

    virtual int SetProfileDuration(double duraton,
                                   double pos1, double pos2,
                                   double vel_max, double acc_max,
                                   double pos_min, double pos_max,
                                   bool research_mode);

    virtual int SetProfileDuration(double duraton,
                                   double pos1, double pos2,
                                   double vel1, double vel2,
                                   double vel_max, double acc_max,
                                   double pos_min, double pos_max,
                                   bool research_mode);

    virtual int SetProfileDuration(double time1, double duraton,
                                   double pos1, double pos2,
                                   double vel1, double vel2,
                                   double vel_max, double acc_max,
                                   double pos_min, double pos_max,
                                   bool research_mode);

    virtual int SetProfileVelocity(double pos1, double pos2,
                                   double vel_max, double acc_max,
                                   double pos_min, double pos_max,
                                   bool research_mode);

    virtual int SetProfileVelocity(double pos1, double pos2, 
                                   double vel1, double vel2, 
                                   double vel_max, double acc_max, 
                                   double pos_min, double pos_max,
                                   bool research_mode);


    virtual int SetProfileVelocity(double time1,
                                   double pos1, double pos2,
                                   double vel1, double vel2,
                                   double vel_max, double acc_max,
                                   double pos_min, double pos_max,
                                   bool research_mode);

    virtual double Duration() const;
    virtual double Pos(double time) const;
    virtual double Vel(double time) const;
    virtual double Acc(double time) const;
    virtual void Write(std::ostream& os) const;
    virtual VelocityProfile* Clone() const;
   // virtual std::string getErrorMsg() const;
    virtual void printCoeffs(void) ;
    //virtual void setLimitValues(double max_vel, double max_acc);
    virtual double calculateDuration(double pos1, double pos2,
                                     double vel1, double vel2,
                                     double vel_max, double acc_max);
private:

/*
        ______________     ___ v_max
       /|            |\
      / |            | \
     /  |            |  \  ___ v_final
    /   |                  ___ v_initial
    +---+------------+--+
    |at |            |dt|
    |      duration     |
    */
    double ap_time_; //acceleration time (at)
    double dp_time_; //deceleration time (dt)
    double bg_time_; //begining time
    double duration_; //movement duration
    double ap_coeff_[3]; //acceleration phase coeffs
    double cvp_coeff_[3]; //constant velocity phase coeffs
    double dp_coeff_[3]; // decceleration phase coeffs 
    std::string error_msg_;
    double epsi_;

    //double vel_max_;
    //double acc_max_;
    //double pos_max_;
    //double pos_min_;

    virtual bool trajectoryIsFeasible(
        double distance, int sign, double vel1, double vel2, double vel_max, double acc_max, bool research_mode, bool duration=false);
    virtual void fillCoeffsWithZeros();
    virtual bool calculateCoeffs(
        double pos1, double pos2, double vel1, double vel2, double vel_max);
    virtual bool maxVelocityIsUnreachable(double distance, double vel1, double vel2, double vel_max, double acc_max, int sign);
    virtual bool trajectoryBreachesPositionLimits(double pos_min, double pos_max);
    virtual void prf(double x, std::string name = "coeff: ");
    virtual bool accMaxIsEnoughForDuration(
        double distance, double vel1, double vel2, double acc_max);
    virtual bool durationIsTooShort();
    virtual bool durationIsTooLong();
    virtual bool isJointAlreadyInPlace(double distance);
    virtual double calculatePhaseTime(double vel_max, double vel, double acc_max);

};
}
#endif // VELOCITYPROFILE_TRAPEZOID_H