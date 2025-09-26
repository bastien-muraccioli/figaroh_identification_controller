#include "FigarohIdentificationController_Initial.h"

#include "../FigarohIdentificationController.h"

void FigarohIdentificationController_Initial::configure(const mc_rtc::Configuration & config) {}

void FigarohIdentificationController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_); 
  int jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(1);
  ctl.compPostureTask->damping(2);
  ctl.compPostureTask->refAccel(Eigen::VectorXd::Zero(jointNumber));
  ctl.compPostureTask->refVel(Eigen::VectorXd::Zero(jointNumber));
  ctl.compPostureTask->target(ctl.postureHome);
  mc_rtc::log::info("[FigarohIdentificationController_Initial] Initial posture");
}

bool FigarohIdentificationController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
  if(ctl.compPostureTask->eval().norm() < 0.05 && !task_achieved_)
  {
    task_achieved_ = true;
    mc_rtc::log::success("[FigarohIdentificationController] Get back to initial posture");
  }
  return false;
}

void FigarohIdentificationController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
}

EXPORT_SINGLE_STATE("FigarohIdentificationController_Initial", FigarohIdentificationController_Initial)
