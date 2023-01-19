#ifndef HARUTO_CONTROLLER__MECANUM_HPP_
#define HARUTO_CONTROLLER__MECANUM_HPP_

#include <Eigen/Dense>

namespace haruto_controller
{
class MecanumDriver
{
  private:
    double x_center_distance_;
    double y_center_distance_;

  public:
    MecanumDriver(double x_center_distance, double y_center_distance);
    Eigen::MatrixXd compute_forward_kinematics(
      double front_left_vel, double front_right_vel, double back_left_vel, double back_right_vel);
    Eigen::MatrixXd compute_inverse_kinematics(
      double x_linear_vel, double y_linear_vel, double z_angular_vel);
};
} // namespace haruto_controller

#endif  // HARUTO_CONTROLLER__MECANUM_HPP_