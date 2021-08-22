#!/usr/bin/env python3

import time
import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64
from haruto_msgs.msg import Velocity
from haruto_msgs.msg import Tick
from haruto_msgs.msg import Reply
from constants import WHEEL_GAP, WHEEL_RADIUS


class HarutoFeedback(object):
    """
        Class that gets feedback of encoder readings from controller and then compute individual wheel velocities and odom 
    """

    def __init__(self):
        self.tick = {'front_left_tick': 0, 'front_right_tick': 0, 'back_left_tick': 0, 'back_right_tick': 0}
        self.speed = {'front_left_actual_speed': 0, 'front_right_actual_speed': 0, 'back_left_actual_speed': 0, 'back_right_actual_speed': 0,
                      'front_left_expected_speed': 0, 'front_right_expected_speed': 0, 'back_left_expected_speed': 0, 'back_right_expected_speed': 0}
        self.time = round(time.time() * 1000)
        self.position = {'x': 0, 'y': 0, 'th': 0}

        self.pid_publisher_front_left = rospy.Publisher('/front_left_wheel/state', Float64, queue_size=10)
        self.pid_publisher_front_right = rospy.Publisher('/front_right_wheel/state', Float64, queue_size=10)
        self.pid_publisher_back_left = rospy.Publisher('/back_left_wheel/state', Float64, queue_size=10)
        self.pid_publisher_back_right = rospy.Publisher('/back_right_wheel/state', Float64, queue_size=10)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.encoder_forward_front_right_encoder_cycle = 0
        self.encoder_forward_front_left_encoder_cycle = 0
        self.encoder_forward_back_right_encoder_cycle = 0
        self.encoder_forward_back_left_encoder_cycle = 0
        self.encoder_reverse_front_right_encoder_cycle = 0
        self.encoder_reverse_front_left_encoder_cycle = 0
        self.encoder_reverse_back_right_encoder_cycle = 0
        self.encoder_reverse_back_left_encoder_cycle = 0

        rospy.init_node('feedback', anonymous=True)
        rospy.loginfo('Intialising haruto feedback node')

    def process_encoder(self, data: Reply):
        """
            Method to 
                1) Get encoder ticks from encoders
                2) The encoder ticks are converted to individual actual wheel velocities
                3) Odom is computed and transform to base link is computed

            :params data: Reply
        """
        data = data.tick
        if round(time.time() * 1000) - self.time >= 1000:
            if self.speed['front_left_expected_speed'] > 0:
                self.speed['front_left_actual_speed'] = abs((data.front_left_tick - self.tick['front_left_tick']) / self.encoder_forward_front_left_encoder_cycle)
                self.speed['back_left_actual_speed'] = abs((data.back_left_tick - self.tick['back_left_tick']) / self.encoder_forward_back_left_encoder_cycle)
            elif self.speed['front_left_expected_speed'] < 0:
                self.speed['front_left_actual_speed'] = abs((data.front_left_tick - self.tick['front_left_tick']) / self.encoder_reverse_front_left_encoder_cycle)
                self.speed['back_left_actual_speed'] = abs((data.back_left_tick - self.tick['back_left_tick']) / self.encoder_reverse_back_left_encoder_cycle)
            else:
                self.speed['front_left_actual_speed'] = 0
                self.speed['back_left_actual_speed'] = 0

            if self.speed['front_right_expected_speed'] > 0:
                self.speed['front_right_actual_speed'] = abs((data.front_right_tick - self.tick['front_right_tick']) / self.encoder_forward_front_right_encoder_cycle)
                self.speed['back_right_actual_speed'] = abs((data.back_right_tick - self.tick['back_right_tick']) / self.encoder_forward_back_right_encoder_cycle)
            elif self.speed['front_right_expected_speed'] < 0:
                self.speed['front_right_actual_speed'] = abs((data.front_right_tick - self.tick['front_right_tick']) / self.encoder_reverse_front_right_encoder_cycle)
                self.speed['back_right_actual_speed'] = abs((data.back_right_tick - self.tick['back_right_tick']) / self.encoder_reverse_back_right_encoder_cycle)
            else:
                self.speed['front_right_actual_speed'] = 0
                self.speed['back_right_actual_speed'] = 0

            self.tick['front_left_tick'] = data.front_left_tick
            self.tick['front_right_tick'] = data.front_right_tick
            self.tick['back_left_tick'] = data.back_left_tick
            self.tick['back_right_tick'] = data.back_right_tick
            self.time = round(time.time() * 1000)

            self.compute_odom()

            setpoint = Float64()
            setpoint = self.speed['front_left_actual_speed']
            self.pid_publisher_front_left.publish(setpoint)
            setpoint = self.speed['front_right_actual_speed']
            self.pid_publisher_front_right.publish(setpoint)
            setpoint = self.speed['back_left_actual_speed']
            self.pid_publisher_back_left.publish(setpoint)
            setpoint = self.speed['back_right_actual_speed']
            self.pid_publisher_back_right.publish(setpoint)

            rospy.loginfo('Encoder FL: {0}, FR: {1}, BL: {2}, BR: {3}'.format(data.front_left_tick, data.front_right_tick, data.back_left_tick, data.back_right_tick))
            rospy.loginfo('Exp Velocity FL: {0}, FR: {1}, BL: {2}, BR: {3}'.format(self.speed['front_left_expected_speed'], self.speed['front_right_expected_speed'], self.speed['back_left_expected_speed'], self.speed['back_right_expected_speed']))


    def process_velocity(self, data: Velocity):
        """
            Method to get expected velocity computed from command controller

            :params data: Velocity
        """
        self.speed['front_left_expected_speed'] = data.front_left_expected_speed
        self.speed['front_right_expected_speed'] = data.front_right_expected_speed
        self.speed['back_left_expected_speed'] = data.back_left_expected_speed
        self.speed['back_right_expected_speed'] = data.back_right_expected_speed

    def compute_odom(self):
        """
            Method to 
                1) Compute odom from individual wheel velocities
                2) Perform transforamtion on odom to base link
        """
        current_time = rospy.Time.now()

        dl = (self.speed['front_left_expected_speed'] + self.speed['back_left_expected_speed']) / 2
        dr = (self.speed['front_right_expected_speed'] + self.speed['back_right_expected_speed']) / 2
        dc  = (dl + dr) / 2
        self.position['th'] = self.position['th'] + ((dl - dr) / WHEEL_GAP)
        self.position['x'] = self.position['x'] + (dc * math.cos(self.position['th']))
        self.position['y'] = self.position['y'] + (dc * math.sin(self.position['th']))

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.position['th'])
        self.odom_broadcaster.sendTransform((self.position['x'], self.position['y'], 0), odom_quat, current_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.position['x'], self.position['y'], 0), Quaternion(*odom_quat))

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(dl, dr, 0), Vector3(0, 0, dc))

        self.odom_pub.publish(odom)

    def set_encoder_tunings(self):
        """
            Method to set encoder tunings values 
        """
        forward_file = open('/home/santosh/Projects/forward.txt', 'r')
        reverse_file = open('/home/santosh/Projects/reverse.txt', 'r')
        
        forward_lines = forward_file.readlines()
        reverse_lines = reverse_file.readlines()

        forward_line = forward_lines[0] if len(forward_lines) > 0 else None
        reverse_line = reverse_lines[0] if len(reverse_lines) > 0 else None
        if forward_line:
            values = forward_line.split(' ')
            if(len(values) == 4):
                self.encoder_forward_front_right_encoder_cycle = int(values[0])
                self.encoder_forward_front_left_encoder_cycle = int(values[1])
                self.encoder_forward_back_right_encoder_cycle = int(values[2])
                self.encoder_forward_back_left_encoder_cycle = int(values[3])
            else:
                rospy.logwarn("Encode tuners for forward mode not set (value missing)")
        else:
            rospy.logwarn("Encode tuners for forward mode not set (file missing)")

        if reverse_line:
            values = reverse_line.split(' ')
            if(len(values) == 4):
                self.encoder_reverse_front_right_encoder_cycle = int(values[0])
                self.encoder_reverse_front_left_encoder_cycle = int(values[1])
                self.encoder_reverse_back_right_encoder_cycle = int(values[2])
                self.encoder_reverse_back_left_encoder_cycle = int(values[3])
            else:
                rospy.logwarn("Encode tuners for reverse mode not set (value missing)")
        else:
            rospy.logwarn("Encode tuners for reverse mode not set (file missing)")

    def start_listening(self):
        """
            Method to start listening (node startup) 
        """
        rospy.Subscriber('/diff_feedback', Reply, self.process_encoder)
        rospy.Subscriber('/diff_velocity', Velocity, self.process_velocity)
        rospy.spin()


if __name__ == '__main__':
    haruto_feedback = HarutoFeedback()
    haruto_feedback.set_encoder_tunings()
    haruto_feedback.start_listening()
