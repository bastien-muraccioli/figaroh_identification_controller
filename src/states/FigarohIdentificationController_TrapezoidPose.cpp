#include "FigarohIdentificationController_TrapezoidPose.h"

#include "../FigarohIdentificationController.h"
#include <cstddef>
// #include <cstddef>

void FigarohIdentificationController_TrapezoidPose::configure(const mc_rtc::Configuration & config) {}

void FigarohIdentificationController_TrapezoidPose::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
  int jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(1);
  ctl.compPostureTask->damping(10);
  ctl.compPostureTask->refAccel(Eigen::VectorXd::Zero(jointNumber));
  ctl.compPostureTask->refVel(Eigen::VectorXd::Zero(jointNumber));
  ctl.compPostureTask->target(ctl.trapezoidHome);
}

bool FigarohIdentificationController_TrapezoidPose::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
  if(ctl.compPostureTask->eval().norm() < 0.05)
  {
    mc_rtc::log::success("[FigarohIdentificationController] Trapezoid posture achieved");
    output("OK");
    return true;
  }
  return false;
}

void FigarohIdentificationController_TrapezoidPose::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FigarohIdentificationController &>(ctl_);
}

EXPORT_SINGLE_STATE("FigarohIdentificationController_TrapezoidPose", FigarohIdentificationController_TrapezoidPose)
