#ifndef HARUTO_CONTROLLER__FOUR_WHEEL_DIFFERENTIAL_HPP_
#define HARUTO_CONTROLLER__FOUR_WHEEL_DIFFERENTIAL_HPP_

#include <Eigen/Dense>

namespace haruto_controller
{
class FourWheelDifferentialDriver
{
  private:
    double wheel_radius_;
    double distance_between_wheels_;

  public:
    FourWheelDifferentialDriver(double wheel_radius, double distance_between_wheels);
    Eigen::MatrixXd compute_forward_kinematics(
      double front_left_vel, double front_right_vel, double back_left_vel, double back_right_vel);
    Eigen::MatrixXd compute_inverse_kinematics(double linear_vel, double angular_vel);
};
} // namespace haruto_controller

#endif  // HARUTO_CONTROLLER__FOUR_WHEEL_DIFFERENTIAL_HPP_