#!/usr/bin/env python3

import rospy
import threading
import time
from std_msgs.msg import Float64
from haruto_msgs.msg import PWM, Velocity


class HarutoConverter(object):
    """ 
        Class which combines given pwm signal and expected speed from different topics to singular topic
    """
    def __init__(self):
        self.pwm_publisher = rospy.Publisher('/diff_pwm', PWM, queue_size=10)
        self.pwm = {
            'front_left_pwm': 0,
            'front_right_pwm': 0,
            'back_left_pwm': 0,
            'back_right_pwm': 0,
            'left_state': 0,
            'right_state': 0
        }
        rospy.init_node('converter', anonymous=True)
        rospy.loginfo('---------- Intialising haruto converter node -----------')

    def convertPWMFrontLeft(self, data: Float64):
        """
            Method to get PWM signal values for front left wheel and store it in memory 

            :params data: Float64
        """
        self.pwm['front_left_pwm'] = int(data.data)

    def convertPWMFrontRight(self, data: Float64):
        """
            Method to get PWM signal values for front right wheel and store it in memory

            :params data: Float64
        """
        self.pwm['front_right_pwm'] = int(data.data)

    def convertPWMBackLeft(self, data: Float64):
        """
            Method to get PWM signal values for back left wheel and store it in memory

            :params data: Float64
        """
        self.pwm['back_left_pwm'] = int(data.data)

    def convertPWMBackRight(self, data: Float64):
        """
            Method to get PWM signal values for back right wheel and store it in memory

            :params data: Float64
        """
        self.pwm['back_right_pwm'] = int(data.data)
    
    def switch_states(self, data: Velocity):
        """
            Method to identify state of rotation for motors 

            :params data: Velocity
        """
        if data.front_left_expected_speed > 0:
            self.pwm['left_state'] = 1
        elif data.front_left_expected_speed < 0:
            self.pwm['left_state'] = 2
        else:
            self.pwm['left_state'] = 0

        if data.front_right_expected_speed > 0:
            self.pwm['right_state'] = 1
        elif data.front_right_expected_speed < 0:
            self.pwm['right_state'] = 2
        else:
            self.pwm['right_state'] = 0

    def converterPWM(self):
        """
            Method which collects all PWM and state data and publish them 
        """
        while True:
            pwm = PWM()
            pwm.front_left_pwm = self.pwm['front_left_pwm']
            pwm.front_right_pwm = self.pwm['front_right_pwm']
            pwm.back_left_pwm = self.pwm['back_left_pwm']
            pwm.back_right_pwm = self.pwm['back_right_pwm']
            pwm.left_state = self.pwm['left_state']
            pwm.right_state = self.pwm['right_state']
            self.pwm_publisher.publish(pwm)
            time.sleep(0.3)

    def start_listening(self):
        """
            Method to start listening (node startup) 
        """
        rospy.Subscriber('front_left_wheel/control_effort', Float64, self.convertPWMFrontLeft)
        rospy.Subscriber('front_right_wheel/control_effort', Float64, self.convertPWMFrontRight)
        rospy.Subscriber('back_left_wheel/control_effort', Float64, self.convertPWMBackLeft)
        rospy.Subscriber('back_right_wheel/control_effort', Float64, self.convertPWMBackRight)
        rospy.Subscriber('diff_velocity', Velocity, self.switch_states)
        rospy.spin()

if __name__ == '__main__':
    haruto_converter = HarutoConverter()
    t = threading.Thread(target=haruto_converter.converterPWM)
    t.daemon = True
    t.start()
    haruto_converter.start_listening()

