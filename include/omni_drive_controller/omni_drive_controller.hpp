#ifndef OMNI_DRIVE_CONTROLLER_HPP_
#define OMNI_DRIVE_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <omni_drive_controller/visibility_control.h>

namespace omni_drive_controller
{
class OmniDriveController : public controller_interface::ControllerInterface
{
public:
  OMNI_DRIVE_CONTROLLER_PUBLIC
  OmniDriveController();

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNI_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
};
} // namespace omni_drive_controller

#endif  // OMNI_DRIVE_CONTROLLER_HPP_
