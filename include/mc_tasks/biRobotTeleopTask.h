#pragma once

// includes
//
#include <tasks/config.hh>

#include <array>

// Eigen
#include <mc_tasks/MetaTask.h>

#include <Tasks/QPSolver.h>
#include <Tasks/QPSolverData.h>
#include <Tasks/Tasks.h>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include "../../include/Tasks/biRobotTeleopTask.h"
#include "../../include/biRobotTeleop/HumanRobotPose.h"
#include <eigen3/Eigen/Core>
#include <sch/CD/CD_Pair.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Point.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI biRobotTeleopTask : public MetaTask
{

public:
  biRobotTeleopTask(const mc_solver::QPSolver & solver,
                    unsigned int r1Index,
                    unsigned int r2Index,
                    const mc_rbdyn::Robot & human1,
                    const mc_rbdyn::Robot & human2,
                    biRobotTeleop::Limbs link1 = biRobotTeleop::Limbs::LeftHand,
                    biRobotTeleop::Limbs link2 = biRobotTeleop::Limbs::RightHand,
                    double stiffness = 1.,
                    double weight = 10.);

  void reset() override
  {
    eval_ = Eigen::Vector6d::Zero();
    speed_ = Eigen::Vector6d::Zero();
    human_1_pose_ = biRobotTeleop::HumanPose("human_pose_1");
    human_2_pose_ = biRobotTeleop::HumanPose("human_pose_2");
    robot_1_pose_links_ = biRobotTeleop::RobotPose();
    robot_2_pose_links_ = biRobotTeleop::RobotPose();
  }

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

  /*! \brief Set the task dimensional weight
   *
   */
  void dimWeight(const Eigen::VectorXd & dimW) override
  {
    task_.dimWeight(dimW);
  }

  Eigen::VectorXd dimWeight() const override
  {
    return task_.dimWeight();
  }

  /*! \brief Select active joints for this task
   *
   * Manipulate \ref dimWeight() to achieve its effect
   */
  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const int rIndex,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {});
  /**
   * @brief Dummy (To implement) use the other one
   *
   * @param solver
   * @param activeJointsName
   * @param activeDofs
   */
  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  /*! \brief Select inactive joints for this task
   *
   * Manipulate \ref dimWeight() to achieve its effect
   */
  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const int rIndex,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {});

  /**
   * @brief Dummy (To implement) use the other one
   *
   * @param solver
   * @param unactiveJointsName
   * @param unactiveDofs
   */
  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  /*! \brief Reset the joint selector effect
   *
   * Reset dimWeight to ones
   */
  void resetJointsSelector(mc_solver::QPSolver & solver, const int rIndex);

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  Eigen::VectorXd eval() const override
  {
    return task_.eval();
  }

  Eigen::VectorXd speed() const override
  {
    return task_.speed();
  }


  double distance(sva::PTransformd vector) 
  {
    return vector.translation().norm();;
  }


  


  void stiffness(double s)
  {
    task_.stiffness(s);
  }

  /** Get the current task's stiffness */
  double stiffness() const
  {
    return task_.stiffness();
  }

  /** Set the task damping, leaving its stiffness unchanged
   *
   * \param damping Task stiffness
   *
   */
  void damping(double d);

  /** Get the task's current damping */
  double damping() const;

  /** Set both stiffness and damping
   *
   * \param stiffness Task stiffness
   *
   * \param damping Task damping
   *
   */
  void setGains(double stiffness, double damping);

  /** Set task's weight */
  void weight(double w)
  {
    weight_ = w;
    task_.dimWeight(Eigen::Vector6d::Ones() * weight_);
  }

  /** Get task's weight */
  double weight() const
  {
    return weight_;
  }

  /** True if the task is in the solver */
  bool inSolver() const;

  void updateHuman(const biRobotTeleop::HumanPose & human_1, const biRobotTeleop::HumanPose & human_2)
  {
    human_1_pose_.updateHumanState(human_1);
    human_2_pose_.updateHumanState(human_2);
  }

  void updateHumanLimbMap(const mc_rtc::Configuration & limbMap)
  {
    human_1_pose_.setLimbMap(limbMap);
    human_2_pose_.setLimbMap(limbMap);
  }

  void updateRobotLinksMap(const biRobotTeleop::RobotPose & robot_1, const biRobotTeleop::RobotPose & robot_2)
  {
    robot_1_pose_links_.load(robot_1);
    robot_2_pose_links_.load(robot_2);
  }

  void updateHumanConfig(const mc_rtc::Configuration & human_1, const mc_rtc::Configuration & human_2)
  {
    human_1_pose_.setCvx(human_1);
    human_2_pose_.setCvx(human_2);
  }

  void setMode(const std::string & mode)
  {
    mode_ = mode;
  }

  const std::string & getMode() const
  {
    return mode_;
  }

  std::vector<biRobotTeleop::HumanPose> getHumanPose()
  {
    return {human_1_pose_, human_2_pose_};
  }

  std::vector<biRobotTeleop::RobotPose> getRobotPose()
  {
    return {robot_1_pose_links_, robot_2_pose_links_};
  }

  biRobotTeleop::Limbs getTargetLink(const int rIndex)
  {
    if(rIndex == r1Index_)
    {
      return link_1_;
    }
    else if(rIndex == r2Index_)
    {
      return link_2_;
    }
    mc_rtc::log::error("[{} , getTargetLink] Robot index {} not in the task, available indexes : {} and {}", name(),
                       rIndex, r1Index_, r2Index_);
    return biRobotTeleop::Limbs::LeftHand;
  }

  void setTargetLink(const int rIndex, const biRobotTeleop::Limbs link)
  {
    if(rIndex == r1Index_)
    {
      link_1_ = link;
    }
    else if(rIndex == r2Index_)
    {
      link_2_ = link;
    }
    else
    {
      mc_rtc::log::error("[{} , setTargetLink] Robot index {} not in the task, available indexes : {} and {}", name(),
                         rIndex, r1Index_, r2Index_);
    }
  }

  std::string getLinkName(const int rIndex, biRobotTeleop::Limbs limb)
  {
    if(rIndex == r1Index_)
    {
      return robot_1_pose_links_.getName(limb);
    }
    if(rIndex == r2Index_)
    {
      return robot_2_pose_links_.getName(limb);
    }
    mc_rtc::log::error("[{} , getLinkName] Robot index {} not in the task, available indexes : {} and {}", name(),
                       rIndex, r1Index_, r2Index_);
    return "";
  }

  /**
   * @brief Get the Relative Transfo from human 1 to robot 2 and from robot 1 to human 2 in that order
   *
   * @return std::vector<sva::PTransformd>
   */
  std::vector<sva::PTransformd> getRelativeTransfo()
  {
    return {X_h1_r2_, X_r1_h2_};
  }

protected:
  void addToSolver(mc_solver::QPSolver & solver) override;

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver &) override;

  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  void removeFromGUI(mc_rtc::gui::StateBuilder &) override;

  void addToLogger(mc_rtc::Logger & logger) override;

  void loadRobotConf(mc_solver::QPSolver & solver, const int rIndex, const mc_rtc::Configuration & conf);

private:
  sch::Matrix4x4 convertTransfo(const sva::PTransformd & X_0_cvx)
  {
    sch::Matrix4x4 m;
    const Eigen::Matrix3d & rot = X_0_cvx.rotation();
    const Eigen::Vector3d & tran = X_0_cvx.translation();

    for(unsigned int i = 0; i < 3; ++i)
    {
      for(unsigned int j = 0; j < 3; ++j)
      {
        m(i, j) = rot(j, i);
      }
    }

    m(0, 3) = tran(0);
    m(1, 3) = tran(1);
    m(2, 3) = tran(2);

    return m;
  }

  Eigen::VectorXd param2VecLocal(const std::vector<std::vector<double>> & param,
                                 const mc_rbdyn::Robot & robot,
                                 const rbd::Jacobian & jac)
  {
    std::vector<double> vec = {};
    for(auto & indx : jac.jointsPath())
    {
      for(auto & p : robot.mbc().alpha[indx])
      {
        vec.push_back(p);
      }
    }
    return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
  }

  /**
   * @brief Compute the offset corresponding to the closest point on the robot to the current human convex sphere
   *
   */
  void getOffset(sva::PTransformd & X_r_rOff,
                 sva::PTransformd & X_h_hOff,
                 sch::CD_Pair & cvx_pair_h_r,
                 const sva::PTransformd & X_0_robotLink,
                 const sva::PTransformd & X_0_humanLink)
  {
    sch::Point3 p1, p2;
    cvx_pair_h_r.getClosestPoints(p1, p2);
    Eigen::Vector3d robot_point_link;
    robot_point_link << p2.m_x, p2.m_y, p2.m_z;
    Eigen::Vector3d human_point_link;
    human_point_link << p1.m_x, p1.m_y, p1.m_z;
    X_r_rOff = sva::PTransformd(X_0_robotLink.rotation(), robot_point_link) * X_0_robotLink.inv();
    X_h_hOff = sva::PTransformd(X_0_humanLink.rotation(), human_point_link) * X_0_humanLink.inv();
  }

  sva::MotionVecd getTargetVel(const mc_rbdyn::Robot & robot, const std::string & link, const Eigen::Vector3d & off)
  {
    const sva::MotionVecd v_l_l = robot.bodyVelB(link);
    const sva::PTransformd X_0_l = robot.bodyPosW(link);
    const sva::PTransformd X_l_t = sva::PTransformd(off);
    return (sva::PTransformd(X_0_l.rotation()).inv() * X_l_t) * v_l_l;
  }

  double getGamma(biRobotTeleop::RobotPose robotLinks,
                  biRobotTeleop::HumanPose humanLinks,
                  bool fromHumanToRobot,
                  biRobotTeleop::Limbs link)
  {
    if(humanLinks.getLength(link) == 0) return 1;
    if(fromHumanToRobot == true)
    {
      // std::cout << link <<" , "<< limb2Str(link)<<" , " << robotLinks.getLength(link)/humanLinks.getLength(link)  <<
      // " fromHumanToRobot: " << fromHumanToRobot <<std::endl;
      return robotLinks.getLength(link) / humanLinks.getLength(link);
    }
    // std::cout << link <<" , "<< limb2Str(link) <<" , "<< humanLinks.getLength(link)/robotLinks.getLength(link) << "
    // fromHumanToRobot : " << fromHumanToRobot <<std::endl;
    else
    {
      return humanLinks.getLength(link) / robotLinks.getLength(link);
    }
  }

  void translateOffset(sva::PTransformd & X_translated,
                       const Eigen::Vector3d & point,
                       const mc_rbdyn::S_ObjectPtr translated_convex,
                       const sva::PTransformd & X_link_translated,
                       double gamma)
  {
    Eigen::Matrix3d gammaMat = Eigen::Matrix3d::Identity(3, 3);
    gammaMat(1, 1) = gamma;
    Eigen::Vector3d point2 = point.transpose() * gammaMat;

    // std::cout << " length after gamma : " << point2[1] << std::endl;

    sva::PTransformd X_0_r2pp = sva::PTransformd(X_link_translated.rotation(), point2) * X_link_translated;
    sch::S_Point projected_point;

    projected_point.setPosition(X_0_r2pp.translation()[0], X_0_r2pp.translation()[1],
                                X_0_r2pp.translation()[2]); // ds le repere monde

    sch::Point3 p1, p2;

    sch::CD_Pair pair_r2_r2p(translated_convex.get(), &projected_point);
    pair_r2_r2p.getClosestPoints(p1, p2);

    Eigen::Vector3d robot2_point;
    robot2_point << p1.m_x, p1.m_y, p1.m_z;

    X_translated = sva::PTransformd(X_link_translated.rotation(), robot2_point) * X_link_translated.inv();
  }

  void OldtranslateOffset(sva::PTransformd & X_translated,
                          sva::PTransformd & X_original,
                          const mc_rbdyn::S_ObjectPtr translated_convex,
                          const sva::PTransformd & X_link_translated,
                          double gamma)
  {
    sva::PTransformd X_0_r2pp = X_original * X_link_translated;
    sch::S_Point projected_point;

    projected_point.setPosition(X_0_r2pp.translation()[0], X_0_r2pp.translation()[1],
                                X_0_r2pp.translation()[2]); // ds le repere monde
    // do the gamma in the method call...

    sch::Point3 p1, p2;

    sch::CD_Pair pair_r2_r2p(translated_convex.get(), &projected_point);
    pair_r2_r2p.getClosestPoints(p1, p2);

    Eigen::Vector3d robot2_point;
    robot2_point << p1.m_x, p1.m_y, p1.m_z;

    X_translated = sva::PTransformd(X_link_translated.rotation(), robot2_point) * X_link_translated.inv();
  }

private:
  tasks::qp::biRobotTeleopTask task_;

  /** True if added to solver */
  bool inSolver_ = false;
  /** Robot handled by the task */
  const mc_rbdyn::Robots & robots_;
  unsigned int r2Index_ = 1;
  unsigned int r1Index_ = 0;
  const mc_rbdyn::Robot & human1_; // estimated human1 robot
  const mc_rbdyn::Robot & human2_; // estimated human2 robot

  unsigned int main_indx_ = 0;

  /** Solver timestep */
  double dt_ = 0.005;

  double weight_ = 0;

  biRobotTeleop::Limbs link_1_;
  biRobotTeleop::Limbs link_2_;

  biRobotTeleop::RobotPose robot_1_pose_links_;
  biRobotTeleop::RobotPose robot_2_pose_links_;
  biRobotTeleop::HumanPose human_1_pose_;
  biRobotTeleop::HumanPose human_2_pose_;
  Eigen::Vector3d robot_2_point_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d human_1_point_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d robot_1_point_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d human_2_point_ = Eigen::Vector3d::Zero();

  sva::PTransformd X_r1_r1p_old_ = sva::PTransformd::Identity();
  sva::PTransformd X_r2_r2p_old_ = sva::PTransformd::Identity();
  sva::PTransformd X_h1_h1p_old_ = sva::PTransformd::Identity();
  sva::PTransformd X_h2_h2p_old_ = sva::PTransformd::Identity();

  // transfo from human_1 to robot_2
  sva::PTransformd X_h1_r2_ = sva::PTransformd::Identity();

  // transfo from robot to human_2
  sva::PTransformd X_r1_h2_ = sva::PTransformd::Identity();

  /** Store the previous eval vector */
  Eigen::VectorXd eval_;
  /** Store the task speed */
  Eigen::VectorXd speed_;

  mc_rtc::gui::ArrowConfig arrowConfig_;

  std::string mode_;
};

using biRobotTelopTaskPtr = std::shared_ptr<biRobotTeleopTask>;

} // namespace mc_tasks
