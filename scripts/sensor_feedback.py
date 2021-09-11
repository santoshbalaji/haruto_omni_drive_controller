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
from constants import WHEEL_GAP, WHEEL_RADIUS, ENCODER_FULL_CYCLE


class HarutoFeedback(object):
    """
        Class that gets feedback of encoder readings from controller and then compute individual wheel velocities and odom 
    """

    def __init__(self):
        self.tick = {
            'front_left_tick': 0, 'front_right_tick': 0, 'back_left_tick': 0, 'back_right_tick': 0
        }
        
        self.speed = {
            'front_left_actual_speed': 0, 'front_right_actual_speed': 0, 'back_left_actual_speed': 0, 'back_right_actual_speed': 0
        }

        self.odom_tick = {
            'front_left_tick': 0, 'front_right_tick': 0, 'back_left_tick': 0, 'back_right_tick': 0
        }
        
        self.odom_speed = {
            'front_left_actual_speed': 0, 'front_right_actual_speed': 0, 'back_left_actual_speed': 0, 'back_right_actual_speed': 0
        }

        self.position = {
            'x': 0, 'y': 0, 'th': 0
        }

        self.time = round(time.time() * 1000)
        self.odom_time = round(time.time() * 1000)

        self.actual_velocity_publisher = rospy.Publisher('/diff_actual_velocity', Velocity, queue_size=10)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.init_node('sensor_feedback', anonymous=True)
        rospy.loginfo('Intialising haruto sensor feedback node')

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
            front_left_tick = - data.front_left_tick
            front_right_tick = data.front_right_tick
            back_left_tick = - data.back_left_tick
            back_right_tick = data.back_right_tick

            self.speed['front_left_actual_speed'] = round(((front_left_tick - self.tick['front_left_tick']) * 2 * 3.14 * WHEEL_RADIUS) / ENCODER_FULL_CYCLE, 3)
            self.speed['front_right_actual_speed'] = round(((front_right_tick - self.tick['front_right_tick']) * 2 * 3.14 * WHEEL_RADIUS) / ENCODER_FULL_CYCLE, 3)
            self.speed['back_left_actual_speed'] = round(((back_left_tick - self.tick['back_left_tick']) * 2 * 3.14 * WHEEL_RADIUS) / ENCODER_FULL_CYCLE, 3)
            self.speed['back_right_actual_speed'] = round(((back_right_tick - self.tick['back_right_tick']) * 2 * 3.14 * WHEEL_RADIUS) / ENCODER_FULL_CYCLE, 3)

            self.tick['front_left_tick'] = front_left_tick
            self.tick['front_right_tick'] = front_right_tick
            self.tick['back_left_tick'] = back_left_tick
            self.tick['back_right_tick'] = back_right_tick
            self.time = round(time.time() * 1000)

            velocity = Velocity()
            velocity.front_left_actual_speed = self.speed['front_left_actual_speed'] * 1
            velocity.front_right_actual_speed = self.speed['front_right_actual_speed'] * 1
            velocity.back_left_actual_speed = self.speed['back_left_actual_speed'] * 1
            velocity.back_right_actual_speed = self.speed['back_right_actual_speed'] * 1

            self.actual_velocity_publisher.publish(velocity)

            rospy.loginfo('FLA: {0}, FRA: {1}, BLA: {2}, BRA: {3}'.format(self.speed['front_left_actual_speed'], self.speed['front_right_actual_speed'], self.speed['back_left_actual_speed'], self.speed['back_right_actual_speed']))
        
        if round(time.time() * 1000) - self.odom_time >= 100:
            front_left_tick = - data.front_left_tick
            front_right_tick = data.front_right_tick
            back_left_tick = - data.back_left_tick
            back_right_tick = data.back_right_tick

            self.odom_speed['front_left_actual_speed'] = round(((front_left_tick - self.odom_tick['front_left_tick']) * 2 * 3.14 * WHEEL_RADIUS) / ENCODER_FULL_CYCLE, 3)
            self.odom_speed['front_right_actual_speed'] = round(((front_right_tick - self.odom_tick['front_right_tick']) * 2 * 3.14 * WHEEL_RADIUS) / ENCODER_FULL_CYCLE, 3)
            self.odom_speed['back_left_actual_speed'] = round(((back_left_tick - self.odom_tick['back_left_tick']) * 2 * 3.14 * WHEEL_RADIUS) / ENCODER_FULL_CYCLE, 3)
            self.odom_speed['back_right_actual_speed'] = round(((back_right_tick - self.odom_tick['back_right_tick']) * 2 * 3.14 * WHEEL_RADIUS) / ENCODER_FULL_CYCLE, 3)

            self.odom_tick['front_left_tick'] = front_left_tick
            self.odom_tick['front_right_tick'] = front_right_tick
            self.odom_tick['back_left_tick'] = back_left_tick
            self.odom_tick['back_right_tick'] = back_right_tick
            self.odom_time = round(time.time() * 1000)

            self.compute_odom()

    def compute_odom(self):
        """
            Method to 
                1) Compute odom from individual wheel velocities
                2) Perform transforamtion on odom to base link
        """
        current_time = rospy.Time.now()

        vl = (self.odom_speed['front_left_actual_speed'] + self.odom_speed['back_left_actual_speed']) / 2
        vr = (self.odom_speed['front_right_actual_speed'] + self.odom_speed['back_right_actual_speed']) / 2
        vc = (WHEEL_RADIUS * (vl + vr)) / 2
        self.position['th'] = self.position['th'] + ((WHEEL_RADIUS * (vr - vl)) / WHEEL_GAP)
        self.position['x'] = self.position['x'] + (vc * math.cos(self.position['th']))
        self.position['y'] = self.position['y'] + (vc * math.sin(self.position['th']))

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.position['th'])
        self.odom_broadcaster.sendTransform((self.position['x'], self.position['y'], 0), odom_quat, current_time, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.position['x'], self.position['y'], 0), Quaternion(*odom_quat))

        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vl, vr, 0), Vector3(0, 0, vc))

        self.odom_pub.publish(odom)

    def start_listening(self):
        """
            Method to start listening (node startup) 
        """
        rospy.Subscriber('/diff_feedback', Reply, self.process_encoder)
        rospy.spin()


if __name__ == '__main__':
    haruto_feedback = HarutoFeedback()
    haruto_feedback.start_listening()
