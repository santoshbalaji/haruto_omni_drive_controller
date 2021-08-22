#!/usr/bin/env python3

import rospy
import os
import tty
import sys
import termios
import _thread
from enum import Enum
from haruto_msgs.msg import Reply
from haruto_msgs.msg import PWM

class TunerMode(Enum):
    IDLE = 1
    FORWARD = 2
    REVERSE = 3
    FORWARD_WRITE = 4
    REVERSE_WRITE = 5

class HarutoEncoderTuner(object):
    """ 
        Class for tuning encoder ticks. (The robot is run for 1m in forward and reverse direction to measure ticks elapsed by encoder)
    """
    def __init__(self):
        self.pwm_publisher = rospy.Publisher('/diff_pwm', PWM, queue_size=10)

        self.tick = {'front_left_tick': 0, 'back_left_tick': 0, 'front_right_tick': 0, 'back_right_tick': 0}
        self.mode = TunerMode.IDLE

        rospy.init_node('encoder_tuner', anonymous=True)
        rospy.loginfo('Intialising haruto encoder tuner node')

    def process_key_events(self):
        """
            Method to listen for keyboard events to start measuring 
            Key s for starting encoder measurements in forward mode 
            key r for starting encoder measurements in reverse mode
            key b for stopping encoder measurements 
        """
        orig_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)
        x = -1
        while x == chr(115) or x == chr(114) or x == chr(98) or x == -1: 
            x = sys.stdin.read(1)[0]
            if x == chr(115):
                self.mode = TunerMode.FORWARD
                pwm = PWM()
                pwm.front_left_pwm = 100
                pwm.back_left_pwm = 100
                pwm.front_right_pwm = 100
                pwm.back_right_pwm = 100
                pwm.left_state = 1
                pwm.right_state = 1
                self.pwm_publisher.publish(pwm)
                rospy.loginfo('starting to record in forward mode')
            elif x == chr(114):
                self.mode = TunerMode.REVERSE
                pwm = PWM()
                pwm.front_left_pwm = 100
                pwm.back_left_pwm = 100
                pwm.front_right_pwm = 100
                pwm.back_right_pwm = 100
                pwm.left_state = 2
                pwm.right_state = 2
                self.pwm_publisher.publish(pwm)
                rospy.loginfo('starting to record in reverse mode')
            elif x == chr(98):
                if self.mode == TunerMode.FORWARD:
                    self.mode = TunerMode.FORWARD_WRITE
                    rospy.loginfo('stopping record in forward mode')
                elif self.mode == TunerMode.REVERSE:
                    self.mode = TunerMode.REVERSE_WRITE 
                    rospy.loginfo('stopping record in reverse mode')
                pwm = PWM()
                pwm.front_left_pwm = 0
                pwm.back_left_pwm = 0
                pwm.front_right_pwm = 0
                pwm.back_right_pwm = 0
                pwm.left_state = 3
                pwm.right_state = 3
                self.pwm_publisher.publish(pwm)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)    

    
    def process_encoder_tuner(self, data: Reply):
        """
            Callback method to get feedback of ticks from encoders 

            :params data: Reply
        """
        if self.mode == TunerMode.FORWARD or self.mode == TunerMode.REVERSE:
            self.tick['front_left_tick'] = abs(data.tick.front_left_tick)
            self.tick['front_right_tick'] = abs(data.tick.front_right_tick)
            self.tick['back_left_tick'] = abs(data.tick.back_left_tick)
            self.tick['back_right_tick'] = abs(data.tick.back_right_tick)

        elif self.mode == TunerMode.FORWARD_WRITE:
            file = open('/home/santosh/Projects/forward.txt', 'w')
            file.write(str(self.tick['front_left_tick']) + ' ' + str(self.tick['back_left_tick']) + ' ' + str(self.tick['front_right_tick']) + ' ' + str(self.tick['back_right_tick']))
            file.close()

            rospy.loginfo('writing to forward file')
            self.mode = TunerMode.IDLE

        elif self.mode == TunerMode.REVERSE_WRITE:
            file = open('/home/santosh/Projects/reverse.txt', 'w')
            file.write(str(self.tick['front_left_tick']) + ' ' + str(self.tick['back_left_tick']) + ' ' + str(self.tick['front_right_tick']) + ' ' + str(self.tick['back_right_tick']))
            file.close()

            rospy.loginfo('writing to reverse file')
            self.mode = TunerMode.IDLE
        
    def start_listening(self):
        """
            Method to start listening (node startup) 
        """
        rospy.Subscriber('/diff_feedback', Reply, self.process_encoder_tuner)
        _thread.start_new_thread(self.process_key_events, ())
        rospy.spin()

if __name__ == '__main__':
    haruto_encoder_tuner = HarutoEncoderTuner()
    haruto_encoder_tuner.start_listening()