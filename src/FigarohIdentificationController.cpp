#include "FigarohIdentificationController.h"
#include <mc_rtc/gui/NumberInput.h>

FigarohIdentificationController::FigarohIdentificationController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{

  // Initialize the constraints
  // selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  // solver().removeConstraintSet(dynamicsConstraint);
  // dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
  //   new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  // solver().addConstraintSet(dynamicsConstraint);

  // postureHome = {{"joint_1", {0}}, {"joint_2", {0}}, {"joint_3", {0}}, {"joint_4", {0}},
  //                  {"joint_5", {0}}, {"joint_6", {0}},  {"joint_7", {0}}};

  jointlimits = config("joint_limits");  
  
  for (const auto& it : jointlimits) {
    const std::string& key = it.first;
    const std::vector<double>& vec = it.second;
    if (!vec.empty()) {
        trapezoidHome[key] = {vec[0]};
    }
  }

  postureHome = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
  {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};

  q_d_ = trapezoidHome;

  solver().removeTask(getPostureTask(robot().name()));
  compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1, 1);
  compPostureTask->reset();
  compPostureTask->target(postureHome);
  compPostureTask->stiffness(1.0);
  compPostureTask->damping(2.0);
  // compPostureTask->stiffness(100.0);
  solver().addTask(compPostureTask);

  // Kinova Gen3 datastore
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("TorqueMode", "Custom");

  stiffness_trapezoid = 0.0;
  damping_trapezoid = 500.0;
  accel_ratio_trapezoid = 0.25;

  // Gui
  gui()->addElement({"Control"},
                 mc_rtc::gui::Button("Position", [this]() { 
                  this->datastore().assign<std::string>("ControlMode", "Position");
                  mc_rtc::log::info("Control mode changed to Position"); 
                }),
                 mc_rtc::gui::Button("Torque", [this]() { 
                  this->datastore().assign<std::string>("ControlMode", "Torque");
                  mc_rtc::log::info("Control mode changed to Torque");
                }),
                mc_rtc::gui::NumberInput("Stiffness", [this]() { return stiffness_trapezoid; },
                [this](double s) { 
                  stiffness_trapezoid = s; 
                  compPostureTask->stiffness(stiffness_trapezoid);
                  mc_rtc::log::info("Stiffness changed to {}", stiffness_trapezoid); 
                }),
                mc_rtc::gui::NumberInput("Damping", [this]() { return damping_trapezoid; },
                [this](double d) { 
                  damping_trapezoid = d; 
                  compPostureTask->damping(damping_trapezoid);
                  mc_rtc::log::info("Damping changed to {}", damping_trapezoid); 
                }),
                mc_rtc::gui::NumberInput("Acceleration ratio", [this]() { return accel_ratio_trapezoid; },
                [this](double a) { 
                  accel_ratio_trapezoid = a; 
                  mc_rtc::log::info("Acceleration ratio changed to {}", accel_ratio_trapezoid); 
                })
              );
  
  // Log
  logger().addLogEntry("FigarohIdentificationController_Stiffness", [this]() { return compPostureTask->stiffness(); });
  logger().addLogEntry("FigarohIdentificationController_Damping", [this]() { return compPostureTask->damping(); });
  logger().addLogEntry("FigarohIdentificationController_ControlMode", [this]() { return datastore().get<std::string>("ControlMode"); });

  mc_rtc::log::success("FigarohIdentificationController init done ");
}

bool FigarohIdentificationController::run()
{
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
}

void FigarohIdentificationController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}
