#include <haruto_controller/two_wheel_differential.hpp>

haruto_controller::TwoWheelDifferentialDriver::TwoWheelDifferentialDriver(
  double wheel_radius, double distance_between_wheels)
{
  this->wheel_radius_ = wheel_radius;
  this->distance_between_wheels_ = distance_between_wheels;
}

Eigen::MatrixXd haruto_controller::TwoWheelDifferentialDriver::compute_forward_kinematics(
  double left_vel, double right_vel)
{
  Eigen::MatrixXd first_set(2, 1);
  Eigen::MatrixXd second_set(2, 2);
  Eigen::MatrixXd third_set(2, 1);
  Eigen::MatrixXd answer_set(2, 1);

  first_set << (this->wheel_radius_ / 2), (this->wheel_radius_ / this->distance_between_wheels_);
  second_set << 1, 1, 1, -1;
  third_set << right_vel, left_vel;
  answer_set = first_set * second_set * third_set;

  return answer_set;
}

Eigen::MatrixXd haruto_controller::TwoWheelDifferentialDriver::compute_inverse_kinematics(
  double linear_vel, double angular_vel)
{
  Eigen::MatrixXd first_set(1, 1);
  Eigen::MatrixXd second_set(2, 2);
  Eigen::MatrixXd third_set(2, 1);
  Eigen::MatrixXd answer_set(2, 1);

  first_set << (1 / (2 * this->wheel_radius_));
  second_set << 1, 1, 1, -1;
  third_set<< linear_vel, this->distance_between_wheels_ * angular_vel;
  answer_set = first_set * second_set * third_set;

  return answer_set;
}
