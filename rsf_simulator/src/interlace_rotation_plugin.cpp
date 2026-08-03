#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <ignition/plugin/Register.hh>

namespace rsf_simulator
{

class InterlaceRotationPlugin
: public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager &) override
  {
    auto model = ignition::gazebo::Model(entity);
    const auto joint_name =
      sdf->Get<std::string>("joint_name", "rsf_hokuyo3d_joint").first;
    joint_ = model.JointByName(ecm, joint_name);
    if (joint_ == ignition::gazebo::kNullEntity) {
      ignerr << "InterlaceRotationPlugin: joint " << joint_name << " not found\n";
      return;
    }
    interlace_ = sdf->Get<int>("interlace", 1).first;
    if (const char * env = std::getenv("RSF_INTERLACE")) {
      interlace_ = std::atoi(env);
    }
    if (interlace_ < 1 || interlace_ > 20) {
      ignwarn << "InterlaceRotationPlugin: interlace " << interlace_
              << " out of range [1, 20], clamped\n";
      interlace_ = std::clamp(interlace_, 1, 20);
    }
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo & info,
    ignition::gazebo::EntityComponentManager & ecm) override
  {
    if (info.paused || joint_ == ignition::gazebo::kNullEntity || interlace_ <= 1) {
      return;
    }
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    const int index = static_cast<int>((ns / 50000000LL) % interlace_);
    const double angle = index * kHorizontalPitch / interlace_;
    auto position =
      ecm.Component<ignition::gazebo::components::JointPositionReset>(joint_);
    if (position) {
      position->Data() = {angle};
    } else {
      ecm.CreateComponent(
        joint_, ignition::gazebo::components::JointPositionReset({angle}));
    }
    auto velocity =
      ecm.Component<ignition::gazebo::components::JointVelocityReset>(joint_);
    if (velocity) {
      velocity->Data() = {0.0};
    } else {
      ecm.CreateComponent(
        joint_, ignition::gazebo::components::JointVelocityReset({0.0}));
    }
  }

private:
  static constexpr double kScanPeriod = 0.05;
  static constexpr double kHorizontalPitch = 0.10471975511965977;

  ignition::gazebo::Entity joint_{ignition::gazebo::kNullEntity};
  int interlace_{1};
};

}

IGNITION_ADD_PLUGIN(
  rsf_simulator::InterlaceRotationPlugin,
  ignition::gazebo::System,
  rsf_simulator::InterlaceRotationPlugin::ISystemConfigure,
  rsf_simulator::InterlaceRotationPlugin::ISystemPreUpdate)
