/**
 * @brief
 * @date 2024-03-04
 * @copyright Copyright (c) 2024
 */

#ifndef IGN_ROS2_CONTROL__IGN_ROS2_CONTROL_PLUGIN_HPP_
#define IGN_ROS2_CONTROL__IGN_ROS2_CONTROL_PLUGIN_HPP_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace ign_ros2_control {
// Forward declarations.
class IgnitionROS2ControlPluginPrivate;

class IgnitionROS2ControlPlugin : public ignition::gazebo::System,
                                  public ignition::gazebo::ISystemConfigure,
                                  public ignition::gazebo::ISystemPreUpdate,
                                  public ignition::gazebo::ISystemPostUpdate {
public:
  /// \brief Constructor
  IgnitionROS2ControlPlugin();

  /// \brief Destructor
  ~IgnitionROS2ControlPlugin() override;

  // Documentation inherited
  void Configure(
      const ignition::gazebo::Entity& _entity,
      const std::shared_ptr<const sdf::Element>& _sdf,
      ignition::gazebo::EntityComponentManager& _ecm,
      ignition::gazebo::EventManager& _eventMgr) override;

  // Documentation inherited
  void PreUpdate(const ignition::gazebo::UpdateInfo& _info, ignition::gazebo::EntityComponentManager& _ecm) override;

  void PostUpdate(
      const ignition::gazebo::UpdateInfo& _info, const ignition::gazebo::EntityComponentManager& _ecm) override;

private:
  /// \brief Private data pointer.
  std::unique_ptr<IgnitionROS2ControlPluginPrivate> dataPtr;
};
}  // namespace ign_ros2_control

#endif  // IGN_ROS2_CONTROL__IGN_ROS2_CONTROL_PLUGIN_HPP_
