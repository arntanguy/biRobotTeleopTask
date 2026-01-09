#pragma once
#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include "boost_serialization_eigen.h"
#include "motion.h"
#include "transformation.h"
#include "type.h"
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>

namespace biRobotTeleop
{

/**
 * @brief Place holder for the human pose and motion datas.
 * The velocity and acceleration datas should be expressed in a body frame oriented as the world frame
 * The position data are stored as their raw value.
 *
 * Offsets are included to account for a unified frame system
 * It is set such as when the human is standing straight with its arms alongside its body,
 * the frame are all oriented similarly as the world frame
 */
struct HumanPose
{

private:
  transformation pose_; // From the world frame to the link frame (offset is not included in this data)
  transformation previous_pose_;
  motion vel_; // written in the body frame oriented as the world frame
  motion acc_; // written in the body frame oriented as the world frame

  std::string name_ = "";

  // transformation between the body Frame to an offsetted link body frame such as
  // when the arm are alongside the body, all the links frame orientation are matching the world frame,
  // the link frame position should be at the parent joint
  transformation limbs_offset_;
  std::map<Limbs, double> convex_radius_;
  std::map<Limbs, double> convex_length_;
  std::map<Limbs, bool>
      data_online_; // By default at false, can be used to set if a data has not been updated for a long time

  std::map<Limbs, std::string> links_;
  std::map<std::string, Limbs> limbs_;

  std::map<Limbs, double> limb_length_;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & pose_;
    ar & vel_;
    ar & acc_;
    ar & limbs_offset_;
    ar & convex_radius_;
    ar & convex_length_;
    ar & limb_length_;
  }

public:
  std::string contact_limbs = "None";

  HumanPose()
  {
    HumanPose("human");
  }

  HumanPose(std::string name)
  {
    name_ = name;
    const auto I = sva::PTransformd::Identity();
    const auto v = sva::MotionVecd::Zero();
    for(int partInt = Limbs::Head; partInt <= Limbs::RightArm; partInt++)
    {
      Limbs part = static_cast<Limbs>(partInt);
      convex_length_[part] = 1;
      convex_radius_[part] = 1;
      limb_length_[part] = 0;
      data_online_[part] = false;
      pose_.add(part, I);
      previous_pose_.add(part, I);
      limbs_offset_.add(part, I);
      vel_.add(part, v);
      acc_.add(part, v);
    }
  }

  void addDataToGUI(mc_rtc::gui::StateBuilder & gui);

  /**
   * @brief Only add human position value to gui with offsets
   *
   * @param gui
   */
  void addPoseToGUI(mc_rtc::gui::StateBuilder & gui, const bool use_offset = true);

  void addOffsetToGUI(mc_rtc::gui::StateBuilder & gui);

  bool limbActive(const Limbs limb) const
  {
    return data_online_.at(limb);
  }

  void setLimbActiveState(const Limbs limb, const bool state) noexcept
  {
    data_online_[limb] = state;
  }

  void setLimbMap(const mc_rtc::Configuration & config)
  {
    auto setLinkName = [this](Limbs limb, std::string linkName)
    {
      links_[limb] = linkName;
      limbs_[linkName] = limb;
    };
    setLinkName(Limbs::LeftHand, config("left_hand"));
    setLinkName(Limbs::RightHand, config("right_hand"));
    setLinkName(Limbs::LeftArm, config("left_arm"));
    setLinkName(Limbs::RightArm, config("right_arm"));
    setLinkName(Limbs::LeftForearm, config("left_forearm"));
    setLinkName(Limbs::RightForearm, config("right_forearm"));
    setLinkName(Limbs::Pelvis, config("pelvis"));
  }

  void setCvx(const mc_rtc::Configuration & config)
  {
    convex_length_.clear();
    convex_radius_.clear();
    auto handleConvexAndOffset = [this, &config](Limbs limb, std::string limbName, std::string side)
    {
      auto limbC = config(limbName);
      if(limbC.has("length") && limbC.has("radius"))
      {
        mc_rtc::log::info("set convex cylinder for limb {}, limbName {}", limb2Str(limb), limbName);
        convex_length_[limb] = limbC("length");
        convex_radius_[limb] = limbC("radius");
      }

      sva::PTransformd offset = limbC("offset")(side);
      limbs_offset_.add(limb, offset);
    };
    handleConvexAndOffset(LeftArm, "arm", "left");
    handleConvexAndOffset(RightArm, "arm", "right");
    handleConvexAndOffset(LeftForearm, "forearm", "left");
    handleConvexAndOffset(RightForearm, "forearm", "right");
    handleConvexAndOffset(LeftHand, "hand", "left");
    handleConvexAndOffset(RightHand, "hand", "right");

    limbs_offset_.add(Pelvis, config("pelvis")("offset"));
  }

  mc_rbdyn::S_ObjectPtr applyTransformation(const Limbs limb, const sva::PTransformd & X_0_p) const
  {
    sva::PTransformd X_p_p1 = sva::PTransformd::Identity();
    sva::PTransformd X_p_p2 =
        sva::PTransformd(Eigen::Matrix3d::Identity(), Eigen::Vector3d{0, 0, -convex_length_.at(limb)});

    auto p1 = (X_p_p1 * X_0_p).translation();
    auto p2 = (X_p_p2 * X_0_p).translation();
    sch::Scalar r = convex_radius_.at(limb);
    return mc_rbdyn::S_ObjectPtr(
        new sch::S_Cylinder(sch::Point3(p1.x(), p1.y(), p1.z()), sch::Point3(p2.x(), p2.y(), p2.z()), r));
  }

  void updateLimbsLength()
  {

    int arms[] = {Limbs::RightArm, Limbs::LeftArm, Limbs::RightForearm, Limbs::LeftForearm};
    // int foreArms[] = {Limbs::RightForearm, Limbs::LeftForearm} ;

    for(int armInt : arms)
    {
      Limbs arm = static_cast<Limbs>(armInt);
      Limbs arm_1 = static_cast<Limbs>(armInt - 1);
      limb_length_[arm] =
          ((getOffset(arm_1) * getPose(arm_1)).translation() - (getOffset(arm) * getPose(arm)).translation()).norm();
    }
  }

  mc_rbdyn::S_ObjectPtr getConvex(Limbs limb, const mc_rbdyn::Robot & robot) const
  {
    if(convex_length_.count(limb) > 0 && convex_radius_.count(limb) > 0)
    { // we have a radius and length in the convex configuration, use a simple cylinder shape
      // mc_rtc::log::info("get cylinder convex for robot {} and limb: {}", robot.name(), limb2Str(limb));
      return applyTransformation(limb, getOffset(limb) * getPose(limb));
    }
    else
    {
      // mc_rtc::log::info("get convex for robot {} and limb: {}, link: {}", robot.name(), limb2Str(limb),
      // links_.at(limb));
      return robot.convex(links_.at(limb)).second;
    }
  }

  const sva::PTransformd & getPose(Limbs limb) const
  {
    return pose_.get(limb);
  }

  const sva::PTransformd & getPreviousPose(Limbs limb) const
  {
    return previous_pose_.get(limb);
  }

  const sva::PTransformd & getOffset(const Limbs limb) const
  {
    return limbs_offset_.get(limb);
  }

  const transformation & getOffset() const noexcept
  {
    return limbs_offset_;
  }

  void setOffset(const transformation & limbs_offsets)
  {
    limbs_offset_ = limbs_offsets;
  }

  void setOffset(Limbs limb, const sva::PTransformd & off)
  {
    limbs_offset_.add(limb, off);
  }

  /**
   * @brief Get the velocity at the limb, velocity is expressed at the limb oriented as the world frame
   *
   * @param limb
   * @param X_b_bOff
   * @return sva::MotionVecd
   */
  sva::MotionVecd getVel(Limbs limb, const sva::PTransformd & X_b_bOff) const
  {
    const Eigen::Matrix3d & R_0_b = (getOffset(limb) * getPose(limb)).rotation();
    sva::PTransformd X_b_bOff0 =
        sva::PTransformd(Eigen::Matrix3d::Identity(), R_0_b.transpose() * X_b_bOff.translation());
    return X_b_bOff0 * vel_.get(limb);
  }

  /**
   * @brief Get the velocity at the limb, velocity is expressed at the limb oriented as the world frame
   *
   * @param limb
   * @return sva::MotionVecd
   */
  const sva::MotionVecd & getVel(Limbs limb) const
  {
    return vel_.get(limb);
  }

  sva::MotionVecd getAcc(Limbs limb, const sva::PTransformd & X_b_bOff) const
  {
    const Eigen::Matrix3d & R_0_b = (getOffset(limb) * getPose(limb)).rotation();
    const sva::PTransformd X_b_bOff0 =
        sva::PTransformd(Eigen::Matrix3d::Identity(), R_0_b.transpose() * X_b_bOff.translation());
    return X_b_bOff0 * acc_.get(limb);
  }

  /**
   * @brief Get the accel at the limb, accel is expressed at the limb oriented as the world frame
   *
   */
  const sva::MotionVecd & getAcc(Limbs limb) const
  {
    return acc_.get(limb);
  }

  const double & getLength(Limbs limb) const
  {
    return limb_length_.at(limb);
  }

  /**
   * @brief Update the human pose on one limb, this data should not necesserly be in the unified frame, if so the
   * corresponding offset should be identity
   *
   * @param limb
   * @param p
   */
  void setPose(const Limbs limb, const sva::PTransformd & p)
  {
    pose_.add(limb, p);
  }
  void setPreviousPose(const Limbs limb, const sva::PTransformd & p)
  {
    previous_pose_.add(limb, p);
  }

  /**
   * @brief Set the limb velocity
   * The velocity is expressed at in the frame of the body oriented as the world frame
   *
   * @param limb
   * @param vel
   */
  void setVel(Limbs limb, const sva::MotionVecd & vel)
  {
    vel_.add(limb, vel);
  }

  /**
   * @brief Set the limb acceleration
   * The velocity is expressed at in the frame of the body oriented as the world frame
   *
   * @param limb
   * @param vel
   */
  void setAcc(Limbs limb, const sva::MotionVecd & acc)
  {
    acc_.add(limb, acc);
  }

  void name(const std::string & n)
  {
    name_ = n;
  }
  const std::string & name() const noexcept
  {
    return name_;
  }

  void updateHumanState(const HumanPose & human)
  {
    for(int partInt = Limbs::Head; partInt <= Limbs::RightArm; partInt++)
    {
      Limbs limb = static_cast<Limbs>(partInt);
      setPose(limb, human.getPose(limb));
      setVel(limb, human.getVel(limb));
      setAcc(limb, human.getAcc(limb));
      setLimbActiveState(limb, human.limbActive(limb));
    }
  }
};

/**
 * @brief RobotPose allows to generically maps a limb to a robot body.
 * It also includes offset such as this body frame can be expressed in the unified frame
 *
 */
struct RobotPose
{
private:
  std::map<Limbs, std::string> links_;
  std::map<std::string, Limbs> limbs_;
  std::map<Limbs, std::string> convex_;
  std::map<Limbs, double> limb_length_;
  std::map<Limbs, double> links_adjusted_;

  // transformation between the body Frame to an offsetted link body frame such as
  // when the arm are alongside the body, all the links frame orientation are matching the world frame,
  // the link frame position should be at the parent joint
  transformation links_offsets_;
  std::string robot_name_;

  sva::PTransformd X_TargetHuman_TargetRobot_ = sva::PTransformd::Identity();

  void setLinksMap(const std::map<Limbs, std::string> & links_map)
  {
    links_ = links_map;
  }
  void setConvexMap(const std::map<Limbs, std::string> & convex)
  {
    convex_ = convex;
  }

  void setOffset(const transformation & links_offsets)
  {
    links_offsets_ = links_offsets;
  }

  void setAdjustedLength(const std::map<Limbs, double> & links_adjusted)
  {
    links_adjusted_ = links_adjusted;
  }

  void robotName(const std::string & name)
  {
    robot_name_ = name;
  }

public:
  RobotPose()
  {
    const auto I = sva::PTransformd::Identity();
    for(int partInt = Limbs::Head; partInt <= Limbs::RightArm; partInt++)
    {
      Limbs part = static_cast<Limbs>(partInt);
      links_[part] = "";
      convex_[part] = "";
      links_offsets_.add(part, I);
      limb_length_[part] = 0;
      links_adjusted_[part] = 0;
    }
  }

  void addTransfo(const mc_rtc::Configuration & config, std::string mode)
  {

    if(mode == "SimulationSingle")
    {
      X_TargetHuman_TargetRobot_ = config("simu");
    }
    else
    {
      X_TargetHuman_TargetRobot_ = config("vr");
    }

    X_TargetHuman_TargetRobot_ = config("vr");
    mc_rtc::log::info("mode is {}, transfo for robot {} is \n{}", mode, robot_name_,
                      X_TargetHuman_TargetRobot_.rotation());
  }

  sva::PTransformd getTransfo()
  {
    return X_TargetHuman_TargetRobot_;
  }

  void load(const mc_rtc::Configuration & config)
  {

    setNameAndConvex(Limbs::LeftHand, config("left_hand"));
    setNameAndConvex(Limbs::RightHand, config("right_hand"));
    setNameAndConvex(Limbs::LeftArm, config("left_arm"));
    setNameAndConvex(Limbs::RightArm, config("right_arm"));
    setNameAndConvex(Limbs::LeftForearm, config("left_forearm"));
    setNameAndConvex(Limbs::RightForearm, config("right_forearm"));
    setNameAndConvex(Limbs::Pelvis, config("pelvis"));
    setNameAndConvex(Limbs::Head, config("head"));

    if(config("left_hand").has("offset"))
    {
      setOffset(Limbs::LeftHand, config("left_hand")("offset"));
    }
    if(config("right_hand").has("offset"))
    {
      setOffset(Limbs::RightHand, config("right_hand")("offset"));
    }
    if(config("left_arm").has("offset"))
    {
      setOffset(Limbs::LeftArm, config("left_arm")("offset"));
    }
    if(config("right_arm").has("offset"))
    {
      setOffset(Limbs::RightArm, config("right_arm")("offset"));
    }
    if(config("left_forearm").has("offset"))
    {
      setOffset(Limbs::LeftForearm, config("left_forearm")("offset"));
    }
    if(config("right_forearm").has("offset"))
    {
      setOffset(Limbs::RightForearm, config("right_forearm")("offset"));
    }

    if(config("left_forearm").has("adjusted_length"))
    {
      setAdjustedLength(Limbs::LeftForearm, config("left_forearm")("adjusted_length"));
    }

    if(config("left_forearm").has("adjusted_length"))
    {
      setAdjustedLength(Limbs::LeftForearm, config("left_forearm")("adjusted_length"));
    }
  }

  void load(const RobotPose & pose)
  {
    setLinksMap(pose.getLinksMap());
    setConvexMap(pose.getConvexesMap());
    setOffset(pose.getOffset());
    robotName(pose.robotName());
    setAdjustedLength(pose.getAdjustedLength());
  }

  void updateLimbsLength(const mc_rbdyn::Robot & robot)
  {

    int arms[] = {Limbs::RightArm, Limbs::LeftArm, Limbs::RightForearm, Limbs::LeftForearm};
    for(int armInt : arms)
    {
      Limbs arm = static_cast<Limbs>(armInt);
      Limbs arm_1 = static_cast<Limbs>(armInt - 1);

      limb_length_[arm] = ((getOffset(arm_1) * robot.bodyPosW(getName(arm_1))).translation()
                           - (getOffset(arm) * robot.bodyPosW(getName(arm))).translation())
                              .norm()
                          - getAdjustedLength(arm);
    }
  }

  void setNameAndConvex(const Limbs part, const mc_rtc::Configuration & config)
  {
    std::string link;
    config("name", link);
    setName(part, link);
    if(config.has("convex"))
    {
      config("convex", convex_[part]);
    }
    else
    {
      convex_[part] = links_[part];
    }
  }
  void setName(const Limbs part, const std::string & name)
  {
    links_[part] = name;
    limbs_[name] = part;
  }
  void setOffset(const Limbs part, const sva::PTransformd & offset)
  {
    links_offsets_.add(part, offset);
  }

  void setAdjustedLength(const Limbs part, const double & adjusted)
  {
    links_adjusted_[part] = adjusted;
  }

  const std::string & robotName() const noexcept
  {
    return robot_name_;
  }

  const std::string getName(const Limbs part) const
  {
    return links_.at(part);
  }

  const Limbs getLimb(const std::string frame) const
  {
    if(limbs_.find(frame) != limbs_.end())
    {
      return limbs_.at(frame);
    }
    std::cout << "frame not referenced" << std::endl;
    return Limbs::Head;
  }

  const double & getLength(Limbs limb) const
  {
    return limb_length_.at(limb);
  }

  const double & getAdjustedLength(Limbs limb) const
  {
    return links_adjusted_.at(limb);
  }

  const std::map<Limbs, double> & getAdjustedLength() const
  {
    return links_adjusted_;
  }

  const std::string getConvexName(const Limbs part) const
  {
    return convex_.at(part);
  }

  sva::PTransformd getOffset(const Limbs limb) const
  {
    return links_offsets_.get(limb);
  }
  const transformation getOffset() const noexcept
  {
    return links_offsets_;
  }
  const std::map<Limbs, std::string> & getLinksMap() const noexcept
  {
    return links_;
  }
  const std::map<Limbs, std::string> & getConvexesMap() const noexcept
  {
    return convex_;
  }
};

} // namespace biRobotTeleop
