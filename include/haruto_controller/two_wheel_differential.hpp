#ifndef HARUTO_CONTROLLER__TWO_WHEEL_DIFFERENTIAL_HPP_
#define HARUTO_CONTROLLER__TWO_WHEEL_DIFFERENTIAL_HPP_

#include <Eigen/Dense>

namespace haruto_controller
{
class TwoWheelDifferentialDriver
{
  private:
    double wheel_radius_;
    double distance_between_wheels_;

  public:
    TwoWheelDifferentialDriver(double wheel_radius, double distance_between_wheels);
    Eigen::MatrixXd compute_forward_kinematics(double left_vel, double right_vel);
    Eigen::MatrixXd compute_inverse_kinematics(double linear_vel, double angular_vel);
};
} // namespace haruto_controller

#endif  // HARUTO_CONTROLLER__TWO_WHEEL_DIFFERENTIAL_HPP_