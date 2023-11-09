#include <omni_drive_controller/kinematics.hpp>

omni_drive_controller::Kinematics::Kinematics(
  double x_center_distance,
  double y_center_distance,
  double wheel_radius)
{
  this->x_center_distance_ = x_center_distance;
  this->y_center_distance_ = y_center_distance;
  this->wheel_radius_ = wheel_radius;
}

Eigen::MatrixXd omni_drive_controller::Kinematics::compute_forward_kinematics(
  double front_left_vel,
  double front_right_vel,
  double back_left_vel,
  double back_right_vel)
{
  Eigen::MatrixXd first_set(1, 1);
  Eigen::MatrixXd second_set(4, 3);
  Eigen::MatrixXd third_set(1, 4);
  Eigen::MatrixXd answer_set(3, 1);

  first_set << (this->wheel_radius_/4);
  second_set << 1, 1, 1, 1
               -1, 1, 1, -1,
               -1/(this->x_center_distance_ + this->y_center_distance_), 
               1/(this->x_center_distance_ + this->y_center_distance_), 
               -1/(this->x_center_distance_ + this->y_center_distance_), 
               1/(this->x_center_distance_ + this->y_center_distance_);
  third_set << front_left_vel, front_right_vel, back_left_vel, back_right_vel;
  answer_set = first_set * second_set * third_set;
  
  return answer_set;
}

Eigen::MatrixXd omni_drive_controller::Kinematics::compute_inverse_kinematics(
  double x_linear_vel,
  double y_linear_vel,
  double z_angular_vel)
{
  Eigen::MatrixXd first_set(1, 1);
  Eigen::MatrixXd second_set(3, 4);
  Eigen::MatrixXd third_set(3, 1);
  Eigen::MatrixXd answer_set(4, 1);

  first_set << (1/this->wheel_radius_);
  second_set << 1, -1, -(this->x_center_distance_ + this->y_center_distance_),
                1, 1, (this->x_center_distance_ + this->y_center_distance_),
                1, 1, -(this->x_center_distance_ + this->y_center_distance_),
                1, -1, (this->x_center_distance_ + this->y_center_distance_);
  third_set << x_linear_vel, y_linear_vel, z_angular_vel;
  answer_set = first_set * second_set * third_set;

  return answer_set;
}
