#include <omni_drive_controller/omni_drive_controller.hpp>

namespace omni_drive_controller
{
controller_interface::CallbackReturn OmniDriveController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OmniDriveController::command_interface_configuration() const
{
}

controller_interface::InterfaceConfiguration OmniDriveController::state_interface_configuration() const
{
}

controller_interface::return_type OmniDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
}

controller_interface::CallbackReturn OmniDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
}

controller_interface::CallbackReturn OmniDriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
}

controller_interface::CallbackReturn OmniDriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
}

controller_interface::CallbackReturn OmniDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
}

controller_interface::CallbackReturn OmniDriveController::on_error(
  const rclcpp_lifecycle::State &)
{
}

controller_interface::CallbackReturn OmniDriveController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
}

}

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(
  omni_drive_controller::OmniDriveController, controller_interface::ControllerInterface)
