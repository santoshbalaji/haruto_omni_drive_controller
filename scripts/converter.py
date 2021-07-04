#!/usr/bin/env python3

import rospy
import threading
import time
from std_msgs.msg import Float64
from haruto_msgs.msg import PWM, Velocity


class HarutoConverter(object):
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
        self.pwm['front_left_pwm'] = data

    def convertPWMFrontRight(self, data: Float64):
        self.pwm['front_right_pwm'] = data

    def convertPWMBackLeft(self, data: Float64):
        self.pwm['back_left_pwm'] = data

    def convertPWMBackRight(self, data: Float64):
        self.pwm['back_right_pwm'] = data
    
    def switch_states(self, data: Velocity):
        if data.front_left_expected_speed > 0:
            self.pwm['left_state'] = 1
        elif data.front_left_expected_speed < 0:
            self.pwm['left_state'] = -1
        else:
            self.pwm['left_state'] = 0

        if data.front_right_expected_speed > 0:
            self.pwm['right_state'] = 1
        elif data.front_right_expected_speed < 0:
            self.pwm['right_state'] = -1
        else:
            self.pwm['right_state'] = 0

    def converterPWM(self):
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
        rospy.Subscriber('front_left_wheel/control_effort', Float64, self.convertPWMFrontLeft)
        rospy.Subscriber('front_right_wheel/control_effort', Float64, self.convertPWMFrontRight)
        rospy.Subscriber('back_left_wheel/control_effort', Float64, self.convertPWMBackLeft)
        rospy.Subscriber('back_right_wheel/control_effort', Float64, self.convertPWMBackRight)
        rospy.spin()

if __name__ == '__main__':
    haruto_converter = HarutoConverter()
    t = threading.Thread(target=haruto_converter.converterPWM)
    t.daemon = True
    t.start()
    haruto_converter.start_listening()

