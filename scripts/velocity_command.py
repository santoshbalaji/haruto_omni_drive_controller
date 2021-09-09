#!/usr/bin/env python3

import rospy
import math
import _thread
import time
from typing import List
from geometry_msgs.msg import Twist
from haruto_msgs.msg import Velocity, PID, PWM
from constants import WHEEL_GAP, WHEEL_RADIUS, PWM_UP_LIMIT, PWM_DOWN_LIMIT, LINEAR_VELOCITY_MAX_LIMIT, LINEAR_VELOCITY_MIN_LIMIT, ANGULAR_VELOCITY_MAX_LIMIT, ANGULAR_VELOCITY_MIN_LIMIT, PID_SCAN_FREQUENCY, PID_ERROR_STACK


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

def compute_errors(errors: List):
    """
        Utility method to compute summation of errors

        :param errors: List
        :returns int
    """
    total = 0
    if errors:
        for i in range(0, len(errors)):
            total = total + errors[i]
    return total

def update_error(errors: List, error: int):
    """
        Utility method to update error stack with new error value
    """
    if len(errors) == PID_ERROR_STACK:
        del errors[0]
    errors.append(error)
    return errors


class HarutoCommand(object):
    """
        Class which takes up command velocities from navigation stack and converts it to individual wheel velocities with closed loop control (PID)
    """

    def __init__(self):
        self.speed = {
            'front_right_expected_speed': 0, 'front_left_expected_speed': 0, 'back_right_expected_speed': 0, 'back_left_expected_speed': 0,
            'front_right_actual_speed': 0, 'front_left_actual_speed': 0, 'back_right_actual_speed': 0, 'back_left_actual_speed': 0
        }

        self.pid = {
            'pfr': 3, 'ifr': 5, 'dfr': 5,
            'pfl': 3, 'ifl': 5, 'dfl': 5,
            'pbr': 3, 'ibr': 5, 'dbr': 5,
            'pbl': 3, 'ibl': 5, 'dbl': 5
        }

        self.previous_error = {
            'front_left': 0, 'front_right': 0, 'back_left': 0, 'back_right': 0
        }

        self.previous_errors = {
            'front_left': [], 'front_right': [], 'back_left': [], 'back_right': []
        }

        self.current_pwm = {
            'front_left': 0, 'front_right': 0, 'back_left': 0, 'back_right': 0
        }

        self.expected_velocity_publisher = rospy.Publisher('/diff_expected_velocity', Velocity, queue_size=10)
        self.pwm_publisher = rospy.Publisher('/diff_pwm', PWM, queue_size=10)

        rospy.init_node('velocity_command', anonymous=True)
        rospy.loginfo('---------- Intialising haruto velocity command node -----------') 
    
    def process_pid(self, data: PID):
        """
            Callback method to setup pid constants

            :param data: PID
        """
        self.pid['pfr'] = data.pfr
        self.pid['ifr'] = data.ifr
        self.pid['dfr'] = data.dfr
        self.pid['pfl'] = data.pfl
        self.pid['ifl'] = data.ifl
        self.pid['dfl'] = data.dfl
        self.pid['pbr'] = data.pbr
        self.pid['ibr'] = data.ibr 
        self.pid['dbr'] = data.dbr
        self.pid['pbl'] = data.pbl
        self.pid['ibl'] = data.ibl
        self.pid['dbl'] = data.dbl
        rospy.loginfo('---------- PID constans set ---------')
 
    def process_expected_velocity(self, data: Twist):
        """
            Callback method which gets invoked when command velocities are sent by navigation stack and then compute individual velocities

            :param data: Twist
        """
        rospy.loginfo('----------- Linear Velocity: {0}, Angular Velocity: {1}'.format(data.linear.x, data.angular.z))

        linear = min(LINEAR_VELOCITY_MAX_LIMIT, max(LINEAR_VELOCITY_MIN_LIMIT, data.linear.x))
        angular = min(ANGULAR_VELOCITY_MAX_LIMIT, max(ANGULAR_VELOCITY_MIN_LIMIT, data.angular.z))

        left_temp = ((2 * linear) - (angular * WHEEL_GAP)) / (2 * WHEEL_RADIUS)
        right_temp = ((2 * linear) + (angular * WHEEL_GAP)) / (2 * WHEEL_RADIUS)
        if data.linear.x != 0:
            left_temp = map((left_temp * 100), -3000, 3000, -1000, 1000)
            self.speed['front_left_expected_speed'] = round((left_temp / 1000), 2)
            self.speed['back_left_expected_speed'] = self.speed['front_left_expected_speed']

            right_temp = round(map((right_temp * 100), -3000, 3000, -1000, 1000), 2)
            self.speed['front_right_expected_speed'] = round((right_temp / 1000), 2)
            self.speed['back_right_expected_speed'] = self.speed['front_right_expected_speed']
        elif data.angular.z != 0:
            left_temp = map((left_temp * 100), -400, 400, -1000, 1000)
            self.speed['front_left_expected_speed'] = round((left_temp / 1000), 2)
            self.speed['back_left_expected_speed'] = self.speed['front_left_expected_speed']

            right_temp = round(map((right_temp * 100), -400, 400, -1000, 1000), 2)
            self.speed['front_right_expected_speed'] = round((right_temp / 1000), 2)
            self.speed['back_right_expected_speed'] = self.speed['front_right_expected_speed']
        else:
            self.speed['front_left_expected_speed'] = 0
            self.speed['front_right_expected_speed'] = 0
            self.speed['back_left_expected_speed'] = 0
            self.speed['back_right_expected_speed'] = 0

        velocity = Velocity()
        velocity.front_left_expected_speed = self.speed['front_left_expected_speed']
        velocity.front_right_expected_speed = self.speed['front_right_expected_speed']
        velocity.back_left_expected_speed = self.speed['back_left_expected_speed']
        velocity.back_right_expected_speed = self.speed['back_right_expected_speed']
        self.expected_velocity_publisher.publish(velocity)

        rospy.loginfo('FLE: {0}, FRE: {1}, BLE: {2}, BRE:{3}'.format(self.speed['front_left_expected_speed'], self.speed['front_right_expected_speed'], self.speed['back_left_expected_speed'], self.speed['back_right_expected_speed']))

    def process_actual_velocity(self, data: Velocity): 
        """
            Callback method which gets invoked when actual velocities are published 

            :param data: Velocity
        """
        self.speed['front_left_actual_speed'] = data.front_left_actual_speed
        self.speed['front_right_actual_speed'] = data.front_right_actual_speed
        self.speed['back_left_actual_speed'] = data.back_left_actual_speed
        self.speed['back_right_actual_speed'] = data.back_right_actual_speed

        rospy.loginfo('FLA: {0}, FRA: {1}, BLA: {2}, BRA:{3}'.format(self.speed['front_left_actual_speed'], self.speed['front_right_actual_speed'], self.speed['back_left_actual_speed'], self.speed['back_right_actual_speed']))

    def calculate_pid(self):
        """
            Routine method which computes the closed loop feedback output (PID)
        """
        while True:
            if self.speed['front_left_expected_speed'] == 0 and self.speed['front_right_expected_speed'] == 0:
                self.current_pwm['front_left'] = 0
                self.current_pwm['front_right'] = 0
                self.current_pwm['back_left'] = 0
                self.current_pwm['back_right'] = 0

                self.previous_error['front_left'] = 0
                self.previous_error['front_right'] = 0
                self.previous_error['back_left'] = 0
                self.previous_error['back_right'] = 0

                self.previous_errors['front_left'] = []
                self.previous_errors['front_right'] = []
                self.previous_errors['back_left'] = []
                self.previous_errors['back_right'] = []
            else:
                front_left_error = abs(self.speed['front_left_expected_speed']) - abs(self.speed['front_left_actual_speed'])
                front_right_error = abs(self.speed['front_right_expected_speed']) - abs(self.speed['front_right_actual_speed'])
                back_left_error = abs(self.speed['back_left_expected_speed']) - abs(self.speed['back_left_actual_speed'])
                back_right_error = abs(self.speed['back_right_expected_speed']) - abs(self.speed['back_right_actual_speed'])

                self.current_pwm['front_left'] = math.floor(self.current_pwm['front_left'] + (self.pid['pfl'] * front_left_error) + (self.pid['dfl'] * self.previous_error['front_left']) + (self.pid['ifl'] * compute_errors(self.previous_errors['front_left']))) 
                self.current_pwm['front_right'] = math.floor(self.current_pwm['front_right'] + (self.pid['pfr'] * front_right_error) + (self.pid['dfr'] * self.previous_error['front_right']) + (self.pid['ifr'] * compute_errors(self.previous_errors['front_right'])))
                self.current_pwm['back_left'] =  math.floor(self.current_pwm['back_left'] + (self.pid['pbl'] * back_left_error) + (self.pid['dbl'] * self.previous_error['back_left']) + (self.pid['ibl'] * compute_errors(self.previous_errors['back_left'])))
                self.current_pwm['back_right'] = math.floor(self.current_pwm['back_right'] + (self.pid['pbr'] * back_right_error) + (self.pid['dbr'] * self.previous_error['back_right']) + (self.pid['ibr'] * compute_errors(self.previous_errors['back_right'])))

                self.current_pwm['front_left'] = min(max(PWM_DOWN_LIMIT, self.current_pwm['front_left']), PWM_UP_LIMIT)
                self.current_pwm['front_right'] = min(max(PWM_DOWN_LIMIT, self.current_pwm['front_right']), PWM_UP_LIMIT)
                self.current_pwm['back_left'] = min(max(PWM_DOWN_LIMIT, self.current_pwm['back_left']), PWM_UP_LIMIT)
                self.current_pwm['back_right'] = min(max(PWM_DOWN_LIMIT, self.current_pwm['back_right']), PWM_UP_LIMIT)
                
                self.previous_error['front_left'] = front_left_error 
                self.previous_error['front_right'] = front_right_error
                self.previous_error['back_left'] = back_left_error
                self.previous_error['back_right'] = back_right_error

                self.previous_errors['front_left'] = update_error(self.previous_errors['front_left'], front_left_error)
                self.previous_errors['front_right'] = update_error(self.previous_errors['front_right'], front_right_error)
                self.previous_errors['back_left'] = update_error(self.previous_errors['back_left'], back_left_error)
                self.previous_errors['back_right'] = update_error(self.previous_errors['back_right'], back_right_error)

            pwm = PWM()
            pwm.front_left_pwm = self.current_pwm['front_left']
            pwm.front_right_pwm = self.current_pwm['front_right']
            pwm.back_left_pwm = self.current_pwm['back_left']
            pwm.back_right_pwm = self.current_pwm['back_right']
            pwm.left_state = 1 if self.speed['front_left_expected_speed'] > 0 else (2 if self.speed['front_left_expected_speed'] < 0 else 3)
            pwm.right_state = 1 if self.speed['front_right_expected_speed'] > 0 else (2 if self.speed['front_right_expected_speed'] < 0 else 3)
            self.pwm_publisher.publish(pwm)

            rospy.loginfo('FLP: {0}, FRP: {1}, BLP: {2}, BRP: {3}'.format(self.current_pwm['front_left'], self.current_pwm['front_right'], self.current_pwm['back_left'], self.current_pwm['back_right']))

            time.sleep(PID_SCAN_FREQUENCY)

    def start_listening(self):
        """
            Method to be called on start (node startup) 
        """
        rospy.Subscriber('diff_pid', PID, self.process_pid)
        rospy.Subscriber('cmd_vel', Twist, self.process_expected_velocity)
        rospy.Subscriber('diff_actual_velocity', Velocity, self.process_actual_velocity)

        rospy.spin()


if __name__ == '__main__':
    haruto_command = HarutoCommand()
    _thread.start_new_thread(haruto_command.calculate_pid, ())
    haruto_command.start_listening()
