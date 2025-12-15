// associated header
#include "../../include/mc_tasks/biRobotTeleopTask.h"
// includes
// std
#include <cmath>
#include <iterator>
#include <set>

// Eigen
#include <Eigen/Geometry>

// RBDyn
#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_solver/TasksQPSolver.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

namespace mc_tasks
{

biRobotTeleopTask::biRobotTeleopTask(const mc_solver::QPSolver & solver,
                                     unsigned int r1Index,
                                     unsigned int r2Index,
                                     const mc_rbdyn::Robot & human1,
                                     const mc_rbdyn::Robot & human2,
                                     biRobotTeleop::Limbs link1,
                                     biRobotTeleop::Limbs link2,
                                     double stiffness,
                                     double weight)
: robots_(solver.robots()), r1Index_(r1Index), r2Index_(r2Index), human1_(human1), human2_(human2), link_1_(link1),
  link_2_(link2), dt_(solver.dt()), task_(solver.robots().mbs(), r1Index, r2Index, stiffness, weight)
{

  eval_ = this->eval();
  speed_ = Eigen::VectorXd::Zero(eval_.size());
  type_ = "biRobotTeleopTask";
  name_ = std::string("biRobotTeleopTask") + robots_.robot(r1Index_).name() + "_" + robots_.robot(r2Index_).name();

  reset();
}

void biRobotTeleopTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);

  if(config.has("stiffness"))
  {
    this->stiffness(config("stiffness"));
  }
  if(config.has("weight"))
  {
    this->weight(config("weight"));
  }
  if(config.has("name"))
  {
    this->name(config("name"));
  }
  if(config.has("human"))
  {
    human_1_pose_.setCvx(config("human")("convex"));
    human_2_pose_.setCvx(config("human")("convex"));
  }

  if(config.has("robot_1"))
  {
    loadRobotConf(solver, r1Index_, config("robot_1"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "{[]} robot_1 configuration is not available. At least the limb map should be provided", name());
  }
  if(config.has("robot_2"))
  {
    loadRobotConf(solver, r2Index_, config("robot_2"));
    if(config("robot_2").has("isMainRobot"))
    {
      main_indx_ = 1;
    }
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "{[]} robot_2 configuration is not available. At least the limb map should be provided", name());
  }

  if(config.has("arrow"))
  {
    arrowConfig_.fromConfig(config("arrow"));
  }
}

void biRobotTeleopTask::loadRobotConf(mc_solver::QPSolver & solver,
                                      const int rIndex,
                                      const mc_rtc::Configuration & config)
{
  if(config.has("link"))
  {
    setTargetLink(rIndex, biRobotTeleop::str2Limb(config("link")));
  }
  if(config.has("active_joints"))
  {
    selectActiveJoints(solver, rIndex, config("active_joints"));
  }
  if(config.has("unactive_joints"))
  {
    selectUnactiveJoints(solver, rIndex, config("unactive_joints"));
  }
  if(config.has("limb_map"))
  {
    if(rIndex == r1Index_)
    {
      robot_1_pose_links_.load(config("limb_map"));
    }
    else if(rIndex == r2Index_)
    {
      robot_2_pose_links_.load(config("limb_map"));
    }
  }
}

void biRobotTeleopTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver_)
  {
    return;
  }
  inSolver_ = false;
  tasks_solver(solver).removeTask(&task_);
}

void biRobotTeleopTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(inSolver_)
  {
    return;
  }
  inSolver_ = true;
  tasks_solver(solver).addTask(&task_);
}

void biRobotTeleopTask::update(mc_solver::QPSolver &)
{

  const mc_rbdyn::Robot & robot_2 = robots_.robot(r2Index_);
  const mc_rbdyn::Robot & robot_1 = robots_.robot(r1Index_);

  const std::string robot_2_link_name = robot_2_pose_links_.getName(link_2_);
  const std::string robot_1_link_name = robot_1_pose_links_.getName(link_1_);

  const std::string robot_2_cvx_name = robot_2_pose_links_.getConvexName(link_2_);
  const auto robot_2_cvx = robot_2.convex(robot_2_cvx_name);
  auto human_1_cvx = human_1_pose_.getConvex(link_1_, human1_);

  const std::string robot_1_cvx_name = robot_1_pose_links_.getConvexName(link_1_);
  const auto robot_1_cvx = robot_1.convex(robot_1_cvx_name);
  auto human_2_cvx = human_2_pose_.getConvex(link_2_, human2_);

  sva::PTransformd X_h1_h1p = sva::PTransformd::Identity();
  sva::PTransformd X_h2_h2p = sva::PTransformd::Identity();
  sva::PTransformd X_r2_r2p = sva::PTransformd::Identity();
  sva::PTransformd X_r1_r1p = sva::PTransformd::Identity();

  sva::PTransformd X_0_robot2_link = robot_2_pose_links_.getOffset(link_2_) * robot_2.bodyPosW(robot_2_link_name);
  sva::PTransformd X_0_human1_link = human_1_pose_.getOffset(link_1_) * human_1_pose_.getPose(link_1_);
  sva::PTransformd X_0_robot1_link = robot_1_pose_links_.getOffset(link_1_) * robot_1.bodyPosW(robot_1_link_name);
  sva::PTransformd X_0_human2_link = human_2_pose_.getOffset(link_2_) * human_2_pose_.getPose(link_2_);

  if(main_indx_ == 0)
  {
    sch::CD_Pair pair_h1_r2(human_1_cvx.get(), robot_2_cvx.second.get());
    getOffset(X_r2_r2p, X_h1_h1p, pair_h1_r2, X_0_robot2_link, X_0_human1_link);
    X_r1_r1p = X_h1_h1p;
    X_h2_h2p = X_r2_r2p;
    translateOffset(X_r1_r1p, X_h1_h1p, robot_1_cvx.second, X_0_robot1_link);

    translateOffset(X_h2_h2p, X_r2_r2p, human_2_cvx, X_0_human2_link);
  }
  else
  {
    sch::CD_Pair pair_h2_r1(human_2_cvx.get(), robot_1_cvx.second.get());
    getOffset(X_r1_r1p, X_h2_h2p, pair_h2_r1, X_0_robot1_link, X_0_human2_link);
    X_r2_r2p = X_h2_h2p;
    X_h1_h1p = X_r1_r1p;

    translateOffset(X_r2_r2p, X_h2_h2p, robot_2_cvx.second, X_0_robot2_link);

    translateOffset(X_h1_h1p, X_r1_r1p, human_1_cvx, X_0_human1_link);
  }

  X_r1_r1p = X_r1_r1p * robot_1_pose_links_.getOffset(link_1_);
  X_r2_r2p = X_r2_r2p * robot_2_pose_links_.getOffset(link_2_);
  X_h1_h1p = X_h1_h1p * human_1_pose_.getOffset(link_1_);
  X_h2_h2p = X_h2_h2p * human_2_pose_.getOffset(link_2_);

  human_1_point_ = (X_h1_h1p * human_1_pose_.getPose(link_1_)).translation();
  robot_2_point_ = (X_r2_r2p * robot_2.bodyPosW(robot_2_link_name)).translation();
  human_2_point_ = (X_h2_h2p * human_2_pose_.getPose(link_2_)).translation();
  robot_1_point_ = (X_r1_r1p * robot_1.bodyPosW(robot_1_link_name)).translation();

  // unused ?
  X_h1_r2_ = (X_r2_r2p * robot_2.bodyPosW(robot_2_link_name)) * (X_h1_h1p * human_1_pose_.getPose(link_1_)).inv();
  X_r1_h2_ = (X_h2_h2p * human_2_pose_.getPose(link_2_)) * (X_r1_r1p * robot_1.bodyPosW(robot_1_link_name)).inv();

  eval_.segment(3, 3) = robot_2_point_ - human_1_point_ - (human_2_point_ - robot_1_point_);

  speed_.segment(3, 3) = getTargetVel(robot_2, robot_2_link_name, X_r2_r2p.translation()).linear()
                         - human_1_pose_.getVel(link_1_, X_h1_h1p).linear()
                         - (human_2_pose_.getVel(link_2_, X_h2_h2p).linear()
                            - getTargetVel(robot_1, robot_1_link_name, X_r1_r1p.translation()).linear());

  rbd::Jacobian robot_1_jac = rbd::Jacobian(robot_1.mb(), robot_1_link_name, X_r1_r1p.translation());
  const auto & shortJ1 = robot_1_jac.jacobian(robot_1.mb(), robot_1.mbc());
  Eigen::MatrixXd J1 = Eigen::MatrixXd::Zero(6, robot_1.mb().nrDof());
  robot_1_jac.fullJacobian(robot_1.mb(), shortJ1, J1);
  const auto dot_J1 = robot_1_jac.jacobianDot(robot_1.mb(), robot_1.mbc());

  rbd::Jacobian robot_2_jac = rbd::Jacobian(robot_2.mb(), robot_2_link_name, X_r2_r2p.translation());
  const auto & shortJ2 = robot_2_jac.jacobian(robot_2.mb(), robot_2.mbc());
  Eigen::MatrixXd J2 = Eigen::MatrixXd::Zero(6, robot_2.mb().nrDof());
  robot_2_jac.fullJacobian(robot_2.mb(), shortJ2, J2);
  const auto dotJ2 = robot_2_jac.jacobianDot(robot_2.mb(), robot_2.mbc());

  auto dot_q1 = param2VecLocal(robot_1.alpha(), robot_1, robot_1_jac);
  auto dot_q2 = param2VecLocal(robot_2.alpha(), robot_2, robot_2_jac);

  task_.setJacobians(J1, J2);

  task_.normalAcc(dotJ2 * dot_q2 - human_1_pose_.getAcc(link_1_, X_h1_h1p).vector()
                  - (human_2_pose_.getAcc(link_2_, X_h2_h2p).vector() - dot_J1 * dot_q1));

  task_.eval(eval_);
  task_.speed(speed_);
}

void biRobotTeleopTask::addToLogger(mc_rtc::Logger & logger)
{
  MetaTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_eval", this, [this]() { return eval(); });
  logger.addLogEntry(name_ + "_stiffness", this, [this]() { return stiffness(); });
  logger.addLogEntry(name_ + "_speed", this, [this]() -> const Eigen::VectorXd & { return speed_; });
  logger.addLogEntry(name_ + "_weight", this, [this]() -> const double & { return weight_; });
}

void biRobotTeleopTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput(
                     "stiffness & damping", [this]() { return this->stiffness(); },
                     [this](const double & g) { this->stiffness(g); }),
                 mc_rtc::gui::NumberInput(
                     "weight", [this]() { return this->weight(); }, [this](const double & w) { this->weight(w); }));

  gui.addElement({"Tasks", name_, "Details"},
                 mc_rtc::gui::Label("robot 1 : ", [this]() { return robots_.robot(r1Index_).name(); }),
                 mc_rtc::gui::Label("link 1 : ", [this]() { return biRobotTeleop::limb2Str(link_1_); }),

                 mc_rtc::gui::Label("robot 2 : ", [this]() { return robots_.robot(r2Index_).name(); }),
                 mc_rtc::gui::Label("link 2 : ", [this]() { return biRobotTeleop::limb2Str(link_2_); }));

  if(main_indx_ == 0)
  {
    gui.addElement({"Tasks", name_, "Details"},

                   mc_rtc::gui::Arrow(
                       "h1 -> r2 convex distance", arrowConfig_, [this]() { return human_1_point_; },
                       [this]() { return robot_2_point_; }),
                   mc_rtc::gui::Arrow(
                       "r1 -> h2 convex distance", arrowConfig_, [this]() { return robot_1_point_; },
                       [this]() { return human_2_point_; }));
  }
  else
  {
    gui.addElement({"Tasks", name_, "Details"},

                   mc_rtc::gui::Arrow(
                       "r2 -> h1 convex distance", arrowConfig_, [this]() { return robot_2_point_; },
                       [this]() { return human_1_point_; }),
                   mc_rtc::gui::Arrow(
                       "h2 -> r1 convex distance", arrowConfig_, [this]() { return human_2_point_; },
                       [this]() { return robot_1_point_; }));
  }

  gui.addElement({"Tasks", name_, "Robot pose"},
                 mc_rtc::gui::Transform("Robot 1: " + biRobotTeleop::limb2Str(link_1_),
                                        [this]()
                                        {
                                          return robot_1_pose_links_.getOffset(link_1_)
                                                 * robots_.robot(r1Index_).bodyPosW(getLinkName(r1Index_, link_1_));
                                        }),
                 mc_rtc::gui::Transform("Robot 2: " + biRobotTeleop::limb2Str(link_2_),
                                        [this]()
                                        {
                                          return robot_2_pose_links_.getOffset(link_2_)
                                                 * robots_.robot(r2Index_).bodyPosW(getLinkName(r2Index_, link_2_));
                                        }));
}

void biRobotTeleopTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::removeFromGUI(gui);
  gui.removeCategory({"Task", name_});
}

void biRobotTeleopTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                           const int rIndex,
                                           const std::vector<std::string> & activeJointsName,
                                           const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex), activeJointsName, "[" + name() + "::selectActiveJoints]");
  std::vector<std::string> unactiveJoints = {};
  for(const auto & j : robots_.robot(rIndex).mb().joints())
  {
    if(j.dof() && std::find(activeJointsName.begin(), activeJointsName.end(), j.name()) == activeJointsName.end())
    {
      unactiveJoints.push_back(j.name());
    }
  }
  selectUnactiveJoints(solver, rIndex, unactiveJoints);
}
void biRobotTeleopTask::selectActiveJoints(mc_solver::QPSolver &,
                                           const std::vector<std::string> &,
                                           const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
}

void biRobotTeleopTask::selectUnactiveJoints(mc_solver::QPSolver &,
                                             const int rIndex,
                                             const std::vector<std::string> & unactiveJointsName,
                                             const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex), unactiveJointsName, "[" + name() + "::selectUnActiveJoints]");
  mc_rtc::log::info("[{}] adding for rIndex {} unactive joints :", name(), rIndex);
  for(auto & n : unactiveJointsName)
  {
    mc_rtc::log::info(n);
  }
  task_.unactiveJoints(unactiveJointsName, rIndex);
}
void biRobotTeleopTask::selectUnactiveJoints(mc_solver::QPSolver &,
                                             const std::vector<std::string> &,
                                             const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
}

void biRobotTeleopTask::resetJointsSelector(mc_solver::QPSolver & solver, const int rIndex)
{
  selectUnactiveJoints(solver, rIndex, {});
}

void biRobotTeleopTask::resetJointsSelector(mc_solver::QPSolver &) {}

} // namespace mc_tasks

// namespace
// {
//
// static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
//     "biRobotTeleop",
//     [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
//     {
//       std::string robot_1_name = solver.robots().robot().name();
//       if(config("robot_1").has("name"))
//       {
//         config("robot_1")("name", robot_1_name);
//       }
//       std::string robot_2_name = "";
//       if(config("robot_2").has("name"))
//       {
//         config("robot_2")("name", robot_2_name);
//       }
//       else
//       {
//         mc_rtc::log::error_and_throw<std::runtime_error>(
//             "[biRobotTeleopTask] robot_2 name shoulde be provided in configuration");
//       }
//
//       const int r1 = solver.robots().robot(robot_1_name).robotIndex();
//       const int r2 = solver.robots().robot(robot_2_name).robotIndex();
//       auto t = std::make_shared<mc_tasks::biRobotTeleopTask>(solver, r1, r2);
//       t->reset();
//       t->load(solver, config);
//       return t;
//     });
// } // namespace
