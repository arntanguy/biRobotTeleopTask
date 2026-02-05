#include "../../include/biRobotTeleop/HumanRobotDataReceiver.h"

#include <chrono>

namespace biRobotTeleop
{

void HumanRobotDataReceiver::init(const std::string name,
                                  const std::string human_name,
                                  const std::string robot_name,
                                  const std::string & pub,
                                  const std::string & rec)
{
  name_ = name;
  human_name_ = human_name;
  robot_name_ = robot_name;
  h_ = HumanPose(human_name_);
  h_thread_ = HumanPose(human_name_);
  robots_ = mc_rbdyn::Robots::make();
  robots_->load("initial", *mc_rbdyn::RobotLoader().get_robot_module({"JVRC1"}));
  robots_->load("initial_thread", *mc_rbdyn::RobotLoader().get_robot_module({"JVRC1"}));
  connect(rec, pub);
}

// XXX: this is mc_control::ControllerClient::start but with online_count_
// useless?
void HumanRobotDataReceiver::startConnection()
{
  run_ = true;
#ifndef MC_RTC_DISABLE_NETWORK
  sub_th_ = std::thread(
      [this]()
      {
        std::vector<char> buff(65536);
        auto t_last_received = std::chrono::system_clock::now();
        while(run_)
        {
          online_count_ += 1;
          {
            std::lock_guard<std::mutex> lk_copy_state(mutex_copy_);
            run(buff, t_last_received);
          }
          online_thread_ = online_count_ < 1000;
          std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
      });
#endif
}

void HumanRobotDataReceiver::update()
{
  std::lock_guard<std::mutex> lk_copy_state(mutex_copy_);
  std::lock_guard<std::mutex> lk_copy_state_delay(mutex_robot_);
  online_ = online_thread_;
  h_.updateHumanState(h_thread_);
  robotMsg = robotMsg_thread;
  if(robots_->hasRobot(robot_name_))
  {
    updateRobot(robots_->robot(robot_name_), robots_->robot(robot_name_ + "_thread"));
  }
}

void HumanRobotDataReceiver::runAndUpdate()
{

  std::vector<char> buff(65536);
  auto t_last_received = std::chrono::system_clock::now();

  online_count_ += 1;

  run(buff, t_last_received);
  update();

  online_ = online_count_ < 1000;
}

void HumanRobotDataReceiver::robot_msg(const mc_control::ElementId & id, const mc_rtc::gui::RobotMsgData & msg)
{
  if(id.category[0] == "BiRobotTeleop")
  {
    if(id.name == robot_name_)
    {
      robotMsg_thread = msg;
      if(!robots_->hasRobot(robot_name_) && !robots_->hasRobot(robot_name_ + "_thread"))
      {
        std::cout << name_ << " update robot " << robot_name_ << std::endl;
        robots_->load(robot_name_, *mc_rbdyn::RobotLoader().get_robot_module(msg.parameters));
        robots_->load(robot_name_ + "_thread", *mc_rbdyn::RobotLoader().get_robot_module(msg.parameters));
        robots_->removeRobot(robots_->robotIndex("initial"));
        robots_->removeRobot(robots_->robotIndex("initial_thread"));
      }
      if(simulated_delay_ != 0)
      {
        robot_thread_ = std::thread([this, msg]() { updateRobot(robots_->robot(robot_name_ + "_thread"), msg); });
        robot_thread_.detach();
      }
      else
      {
        updateRobot(robots_->robot(robot_name_ + "_thread"), msg);
      }
    }
  }
  std::lock_guard<std::mutex> lock(subscribed_data_mutex_);
  for(auto & subs : subscribed_id_)
  {
    if(subs.second.checkId(id))
    {
      subscribed_data_[subs.first] = msg;
    }
  }
}

void HumanRobotDataReceiver::checkbox(const mc_control::ElementId & id, bool s)
{
  if(id.category[0] == "BiRobotTeleop")
  {
    if(id.name == "Online")
    {
      online_count_ = 0;
    }
    else if(id.category.size() >= 4)
    {
      if(id.category[2] == human_name_ && id.category.back() == "Online")
      {
        const auto limb = str2Limb(id.name);
        h_thread_.setLimbActiveState(limb, s);
        // std::cout << "limb " << id.name << " is active " << s << std::endl;
      }
    }
  }
  std::lock_guard<std::mutex> lock(subscribed_data_mutex_);
  for(auto & subs : subscribed_id_)
  {
    if(subs.second.checkId(id))
    {
      subscribed_data_[subs.first] = s;
    }
  }
}

void HumanRobotDataReceiver::transform(const mc_control::ElementId & id,
                                       const mc_control::ElementId & /*requestId*/,
                                       bool /*ro */,
                                       const sva::PTransformd & X)
{
  if(id.category.size() == 4)
  {
    if(id.category[1] == "Human Pose" && id.category[2] == human_name_ && id.category[3] == "Transforms")
    {
      auto limb = str2Limb(id.name.substr(0, id.name.find("_transform")));
      h_thread_.setPose(limb, X);
    }
  }
  std::lock_guard<std::mutex> lock(subscribed_data_mutex_);
  for(auto & subs : subscribed_id_)
  {
    if(subs.second.checkId(id))
    {
      subscribed_data_[subs.first] = X;
    }
  }
}

void HumanRobotDataReceiver::array_input(const mc_control::ElementId & id,
                                         const std::vector<std::string> & /*labels*/,
                                         const Eigen::VectorXd & data)
{

  if(id.category.size() == 4)
  {
    if(id.category[1] == "Human Pose" && id.category[2] == human_name_ && id.category[3] == "Velocity")
    {
      auto limb = str2Limb(id.name);
      assert(data.size() == 6);
      h_thread_.setVel(limb, sva::MotionVecd(data.segment(0, 6)));
    }
  }
  std::lock_guard<std::mutex> lock(subscribed_data_mutex_);
  for(auto & subs : subscribed_id_)
  {
    if(subs.second.checkId(id))
    {
      subscribed_data_[subs.first] = data;
    }
  }
}

void HumanRobotDataReceiver::label(const mc_control::ElementId & id, const std::string & label)
{
  std::lock_guard<std::mutex> lock(subscribed_data_mutex_);
  // const mc_control::ElementId id_f(id.category,id.name.substr(0,id.name.find("_transform")))
  for(auto & subs : subscribed_id_)
  {
    if(subs.second.checkId(id))
    {
      subscribed_data_[subs.first] = label;
    }
  }
}

void HumanRobotDataReceiver::array_label(const mc_control::ElementId & id,
                                         const std::vector<std::string> &,
                                         const Eigen::VectorXd & data)
{

  std::lock_guard<std::mutex> lock(subscribed_data_mutex_);
  for(auto & subs : subscribed_id_)
  {
    if(subs.second.checkId(id))
    {
      subscribed_data_[subs.first] = data;
    }
  }
}

void HumanRobotDataReceiver::updateRobot(mc_rbdyn::Robot & robot, const mc_rtc::gui::RobotMsgData & msg)
{
  const int dur = static_cast<int>(simulated_delay_ * 1e3);
  if(dur == 0)
  {
    robot.mbc().q = rbd::vectorToParam(robot.mb(), msg.q);
    robot.mbc().alpha = rbd::vectorToDof(robot.mb(), msg.alpha);
    robot.mbc().alphaD = rbd::vectorToDof(robot.mb(), msg.alphaD);
    robot.posW(msg.posW);
    return;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(dur));

  std::lock_guard<std::mutex> lk_copy_state(mutex_robot_);
  robot.mbc().q = rbd::vectorToParam(robot.mb(), msg.q);
  robot.mbc().alpha = rbd::vectorToDof(robot.mb(), msg.alpha);
  robot.mbc().alphaD = rbd::vectorToDof(robot.mb(), msg.alphaD);
  robot.posW(msg.posW);
}

void HumanRobotDataReceiver::updateRobot(mc_rbdyn::Robot & robot_target, const mc_rbdyn::Robot & robot_data)
{
  robot_target.mbc().q = robot_data.mbc().q;
  robot_target.mbc().alpha = robot_data.mbc().alpha;
  robot_target.mbc().alphaD = robot_data.mbc().alphaD;
  robot_target.posW(robot_data.posW());
}

} // namespace biRobotTeleop
