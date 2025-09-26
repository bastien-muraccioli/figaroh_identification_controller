#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantPostureTask.h>

#include "api.h"

struct FigarohIdentificationController_DLLAPI FigarohIdentificationController : public mc_control::fsm::Controller
{
  FigarohIdentificationController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::map<std::string, std::vector<double>> jointlimits;
  std::map<std::string, std::vector<double>> q_d_;
  
  // Targets
  std::map<std::string, std::vector<double>> postureHome;
  std::map<std::string, std::vector<double>> trapezoidHome;
  // Tasks
  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;

  double stiffness_trapezoid;
  double damping_trapezoid;
  double accel_ratio_trapezoid;

private:
  mc_rtc::Configuration config_;
};
