#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from haruto_msgs.msg import Velocity
from constants import WHEEL_GAP, WHEEL_RADIUS


def map(value: float, from_min: int, from_max: int, to_min: int, to_max: int):
    """
        Utility method to nomralize provided values from a range to a specific range of integers provided

        :param value: float
        :param from_min: int
        :param from_max: int
        :param to_min: int
        :param to_max: int    
    """
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min


class HarutoCommand(object):
    """
        Class which takes up command velocities from navigation stack and converts it to individual velocities which
        will be fed to ROS PID package
    """

    def __init__(self):
        self.speed = dict()
        self.velocity_publisher = rospy.Publisher('/diff_velocity', Velocity, queue_size=10)
        self.pid_publisher_front_left = rospy.Publisher('/setpoint_front_left_wheel', Float64, queue_size=10)
        self.pid_publisher_front_right = rospy.Publisher('/setpoint_front_right_wheel', Float64, queue_size=10)
        self.pid_publisher_back_left = rospy.Publisher('/setpoint_back_left_wheel', Float64, queue_size=10)
        self.pid_publisher_back_right = rospy.Publisher('/setpoint_back_right_wheel', Float64, queue_size=10)

        rospy.init_node('command', anonymous=True)
        rospy.loginfo('---------- Intialising haruto command node -----------') 

    def process_velocity(self, data: Twist):
        """
            Callback method which gets invoked when command velocities are sent by navigation stack and individual wheel velocities 
            are sent to PID controller for generating PWM signals 

            :param data: Twist
        """
        rospy.loginfo('----------- Linear Velocity: {0}, Angular Velocity: {1}'.format(data.linear.x, data.angular.z))
        temp = (data.linear.x - ((data.angular.z * WHEEL_GAP) / 2)) / WHEEL_RADIUS
        temp = map((temp * 100), -3650, 3650, -800, 800)
        self.speed['front_left_expected_speed'] = temp / 1000
        self.speed['back_left_expected_speed'] = self.speed['front_left_expected_speed']

        temp = (data.linear.x + ((data.angular.z * WHEEL_GAP) / 2)) / WHEEL_RADIUS
        temp = map((temp * 100), -3650, 3650, -800, 800)
        self.speed['front_right_expected_speed'] = temp / 1000
        self.speed['back_right_expected_speed'] = self.speed['front_right_expected_speed']

        setpoint = Float64()
        setpoint = abs(self.speed['front_left_expected_speed'])
        self.pid_publisher_front_left.publish(setpoint)
        setpoint = abs(self.speed['front_right_expected_speed'])
        self.pid_publisher_front_right.publish(setpoint)
        setpoint = abs(self.speed['back_left_expected_speed'])
        self.pid_publisher_back_left.publish(setpoint)
        setpoint = abs(self.speed['back_right_expected_speed'])
        self.pid_publisher_back_right.publish(setpoint)

        velocity = Velocity()
        velocity.front_left_expected_speed = self.speed['front_left_expected_speed']
        velocity.front_right_expected_speed = self.speed['front_right_expected_speed']
        velocity.back_left_expected_speed = self.speed['back_left_expected_speed']
        velocity.back_right_expected_speed = self.speed['back_right_expected_speed']
        self.velocity_publisher.publish(velocity)

    def start_listening(self):
        """
            Method to start listening (node startup) 
        """
        rospy.Subscriber('cmd_vel', Twist, self.process_velocity)
        rospy.spin()


if __name__ == '__main__':
    haruto_command = HarutoCommand()
    haruto_command.start_listening()
