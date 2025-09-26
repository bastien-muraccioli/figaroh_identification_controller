#include "FigarohIdentificationController_Trapezoid.h"
#include <mc_rtc/logging.h>

#include "../FigarohIdentificationController.h"
#include <cstddef>
// #include <cstddef>

void FigarohIdentificationController_Trapezoid::configure(const mc_rtc::Configuration & config) {}

void FigarohIdentificationController_Trapezoid::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
  ctl.compPostureTask->target(ctl.trapezoidHome);
  ctl.compPostureTask->stiffness(ctl.stiffness_trapezoid);
  ctl.compPostureTask->damping(ctl.damping_trapezoid);
  jointNumber_ = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();
  round_trip_ = 6;
  round_trip_counter_ = 0;
  velocity_ = Eigen::VectorXd(jointNumber_);
  accel_ = Eigen::VectorXd(jointNumber_);
  q_min_ = Eigen::VectorXd(jointNumber_);
  q_max_ = Eigen::VectorXd(jointNumber_);
  delta_q_ = Eigen::VectorXd(jointNumber_);
  accel_ratio_ = ctl.accel_ratio_trapezoid;
  vel_max_ = Eigen::VectorXd(jointNumber_);
  ctl.q_d_ = ctl.trapezoidHome;
  q_d_vector_ = Eigen::VectorXd(jointNumber_);
  size_t i = 0;
  for(const auto& [joint, pos]: ctl.q_d_)
  {
    q_d_vector_[i] = pos[0];
    i++;
  }


  accel_constant_ = Eigen::VectorXd(jointNumber_);
  time_counter_ = 0.0;
  sign_ = 1.0;
  speed_factor_ = 0.2;
  // tf0_nominal_ = 1.2;
  tf0_nominal_ = 3.1;
  i = 0;
  for(const auto& [joint, limits]: ctl.jointlimits)
  {
    if(i>jointNumber_)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
        "[FigarohIdentificationController_Trapezoid] The number of joints in the configuration file is not equal to the number of joints in the robot module");
    }
    q_min_[i] = limits[0];
    q_max_[i] = limits[1];
    delta_q_[i] = std::abs(q_max_[i] - q_min_[i]);
    i++;
  }
  mc_rtc::log::info("[FigarohIdentificationController_Trapezoid] q_min_ {}, q_max_ {}", q_min_.transpose(), q_max_.transpose());

  // Calculate the required time based on target position and peak speed for the first joint
  vel_max_ = speed_factor_ * delta_q_ / tf0_nominal_;
  total_time_ = delta_q_[0] / vel_max_[0] / (1 - accel_ratio_);
  tf_acc_ = total_time_ * accel_ratio_; // Acceleration time
  tf_const_ = total_time_ - 2 * tf_acc_; // Constant speed time
  accel_constant_ = vel_max_ / tf_acc_;
  
  mc_rtc::log::info("[FigarohIdentificationController_Trapezoid] round_trip_counter_ {}, speed_factor_ {}, tf_acc_ {}, tf_const_ {} , total_time_ {}, vel_max_ {}, accel_constant_ {}",
    round_trip_counter_, speed_factor_, tf_acc_, tf_const_, total_time_, vel_max_.transpose(), accel_constant_.transpose());
  
  addLog(ctl_);  
}

bool FigarohIdentificationController_Trapezoid::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
  if(round_trip_counter_ > round_trip_ -1)
  {
    mc_rtc::log::info("[FigarohIdentificationController_Trapezoid] Calibration round trip completed");
    ctl.compPostureTask->reset();
    output("OK");
    return true;
  }
  computeTrapezoidVelocity(ctl_);
  ctl.compPostureTask->refAccel(accel_);
  ctl.compPostureTask->refVel(velocity_);
  ctl.compPostureTask->target(ctl.q_d_);
  time_counter_ += ctl.timeStep;
  if(time_counter_ > total_time_)
  {
    time_counter_ = 0.0;
    if(sign_ < 0.0)
    {
      sign_ = 1.0;
      round_trip_counter_++;
      if(round_trip_counter_ > round_trip_ -1)
      {
        mc_rtc::log::info("[FigarohIdentificationController_Trapezoid] Calibration round trip completed");
        ctl.compPostureTask->reset();
        output("OK");
        return true;
      }
      // Increase speed by 0.2 every two round trips
      speed_factor_ = 0.2 *((round_trip_counter_ / 2) + 1);
      vel_max_ = speed_factor_ * delta_q_ / tf0_nominal_;
      // Total time based on the first joint
      total_time_ = delta_q_[0] / vel_max_[0] / (1 - accel_ratio_); 
      tf_acc_ = total_time_ * accel_ratio_; // Acceleration time
      tf_const_ = total_time_ - 2 * tf_acc_; // Constant speed time
      accel_constant_ = vel_max_ / tf_acc_;
      mc_rtc::log::info("[FigarohIdentificationController_Trapezoid] round_trip_counter_ {}, speed_factor_ {}, tf_acc_ {}, tf_const_ {} , total_time_ {}, vel_max_ {}, accel_constant_ {}",
        round_trip_counter_, speed_factor_, tf_acc_, tf_const_, total_time_, vel_max_.transpose(), accel_constant_.transpose());
    }
    else
    {
      sign_ = -1.0;
    }
  }
  return false;
}

void FigarohIdentificationController_Trapezoid::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
  ctl.q_d_ = ctl.trapezoidHome;
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_velocity");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_accel");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_q_d");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_time_counter");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_round_trip_counter");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_round_trip");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_sign");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_speed_factor");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_tf_acc");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_tf_const");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_tf0_nominal");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_total_time");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_delta_q");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_q_min");
  ctl.logger().removeLogEntry("FigarohIdentificationController_Trapezoid_q_max");
  
}

void FigarohIdentificationController_Trapezoid::computeTrapezoidVelocity(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  size_t i = 0;
  for (const auto& [joint, pos]: ctl.q_d_)
  {
    if(i>jointNumber_)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
        "[FigarohIdentificationController_Trapezoid] The number of joints in the configuration file is not equal to the number of joints in the robot module");
    }
    double a_t = 0.0;
    double v_t = 0.0;
    double q_t = 0.0;
    double q_zero = 0.0;
    if(sign_ < 0.0){
      q_zero = q_max_[i];
    }
    else {
      q_zero = q_min_[i];
    }

    if(time_counter_ <= tf_acc_)
    {
      // Acceleration phase
      a_t = accel_constant_[i] * sign_;
      v_t = accel_constant_[i] * time_counter_ * sign_;
      q_t = q_zero + 0.5 * accel_constant_[i] * time_counter_ * time_counter_ * sign_;
    }
    else if(time_counter_ > tf_acc_ && time_counter_ <= tf_const_ + tf_acc_)
    {
      // Constant speed phase
      a_t = 0.0;
      v_t = vel_max_[i] * sign_;
      q_t = q_zero + 0.5 * accel_constant_[i] * tf_acc_ * tf_acc_ * sign_ + vel_max_[i] * (time_counter_ - tf_acc_) * sign_;
    }
    else if(time_counter_ > tf_const_ + tf_acc_)
    {
      // Deceleration phase
      double t_dec = time_counter_ - (tf_acc_ + tf_const_);
      a_t = -accel_constant_[i] * sign_;
      v_t = vel_max_[i] * sign_ - accel_constant_[i] * t_dec * sign_;
      q_t = q_zero + 0.5 * accel_constant_[i] * tf_acc_ * tf_acc_ * sign_ +
      vel_max_[i] * tf_const_ * sign_ + 
      vel_max_[i] * t_dec * sign_ - 0.5 * accel_constant_[i] * t_dec * t_dec * sign_;
    }

    q_d_vector_[i] = q_t;
    ctl.q_d_[joint][i] = q_t;
    velocity_[i] = v_t;
    accel_[i] = a_t;
    i++;
  }
  // mc_rtc::log::info("[FigarohIdentificationController_Trapezoid] velocity_ {}, accel_ {}", 
  //   velocity_.transpose(), accel_.transpose());
}

void FigarohIdentificationController_Trapezoid::addLog(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_velocity",
    [&, this]() { return this->velocity_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_accel",
    [&, this]() { return this->accel_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_q_d",
    [&, this]() { return this->q_d_vector_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_time_counter",
    [&, this]() { return this->time_counter_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_round_trip_counter",
    [&, this]() { return this->round_trip_counter_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_round_trip",
    [&, this]() { return this->round_trip_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_sign",
    [&, this]() { return this->sign_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_speed_factor",
    [&, this]() { return this->speed_factor_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_tf_acc",
    [&, this]() { return this->tf_acc_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_tf_const",
    [&, this]() { return this->tf_const_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_tf0_nominal",
    [&, this]() { return this->tf0_nominal_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_total_time",
    [&, this]() { return this->total_time_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_delta_q",
    [&, this]() { return this->delta_q_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_q_min",
    [&, this]() { return this->q_min_; });
  ctl.logger().addLogEntry("FigarohIdentificationController_Trapezoid_q_max",
    [&, this]() { return this->q_max_; });

}

EXPORT_SINGLE_STATE("FigarohIdentificationController_Trapezoid", FigarohIdentificationController_Trapezoid)
