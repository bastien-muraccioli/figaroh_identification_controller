#pragma once

#include <mc_control/fsm/State.h>

struct FigarohIdentificationController_SerialTrapezoid : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void computeTrapezoidVelocity(mc_control::fsm::Controller & ctl);

  void addLog(mc_control::fsm::Controller & ctl);

private:
  size_t jointNumber_;
  size_t round_trip_;
  size_t round_trip_counter_;
  size_t joint_counter_;
  Eigen::VectorXd velocity_;
  Eigen::VectorXd accel_;
  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;
  Eigen::VectorXd delta_q_; // Difference between max and min position
  double accel_ratio_; // Ratio of acceleration to maximum velocity
  Eigen::VectorXd vel_max_; //2.0944; // Maximum velocity
  // std::map<std::string, std::vector<double>> q_d_;
  Eigen::VectorXd q_d_vector_; // Target position for log purpose
  // Eigen::VectorXd q_d_zero_; // Position before trapezoid velocity
  double total_time_;
  double tf_acc_; // Acceleration time
  double tf_const_; // Constant speed time
  Eigen::VectorXd accel_constant_;
  double time_counter_;
  double sign_;
  double speed_factor_;
  double tf0_nominal_;
  
};
