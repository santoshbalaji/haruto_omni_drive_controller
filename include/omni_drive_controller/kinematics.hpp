#ifndef OMNI_DRIVE_CONTROLLER__KINEMATICS_HPP_
#define OMNI_DRIVE_CONTROLLER__KINEMATICS_HPP_

#include <Eigen/Dense>

namespace omni_drive_controller
{
class Kinematics
{
  private:
    double x_center_distance_;
    double y_center_distance_;
    double wheel_radius_;

  public:
    Kinematics(
      double x_center_distance,
      double y_center_distance,
      double wheel_radius);
    Eigen::MatrixXd compute_forward_kinematics(
      double front_left_vel,
      double front_right_vel,
      double back_left_vel,
      double back_right_vel);
    Eigen::MatrixXd compute_inverse_kinematics(
      double x_linear_vel,
      double y_linear_vel,
      double z_angular_vel);
};
} // namespace omni_drive_controller

#endif  // OMNI_DRIVE_CONTROLLER__KINEMATICS_HPP_
