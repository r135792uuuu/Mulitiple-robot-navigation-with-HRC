#!/usr/bin/env python

import rospy

import math
import tf
from tf import transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf import broadcaster
from dynamic_reconfigure.server import Server
from abc_swarm.cfg import tf_pidConfig
import numpy as np
from simple_pid import PID


pid_linear = PID(0.5, 0.0, 0.0)
pid_linear.output_limits = (-0.8, 0.8)
pid_angular = PID(1, 0.0, 0.0)
pid_angular.output_limits = (-1.5, 1.5)

k_1 = 1
k_2 = 1
d = 0.1
w_leader = 0
v_leader = 0



def pid_cb(config, level):
    rospy.loginfo("""Reconfigure Request: {linear_kp}, {linear_ki}, {linear_kd}, {angular_kp}, {angular_ki}, {angular_kd}""".format(**config))
    # pid_linear.tunings = [float(config.get(key)) for key in ['linear_kp', 'linear_ki', 'linear_kd']]
    # pid_angular.tunings = [float(config.get(key)) for key in ['angular_kp','angular_ki','angular_kd']]
    return config

def odom_cb(msg):
    w_leader = msg.twist.twist.angular.z
    v_leader = msg.twist.twist.linear.x
    

if __name__ == '__main__':
    rospy.init_node('tianbot_mini_tf_listener')
    leader_robot_name = rospy.get_param('~leader_robot_name')
    follower_robot_name = rospy.get_param('~follower_robot_name')
    target_frame = rospy.get_param('~target_frame',leader_robot_name+"/base_link")
    theta0 = rospy.get_param('~expected_theta', np.pi)
    L0 = rospy.get_param('~expected_distance', 1)
    d = rospy.get_param('~front_distance', 0.1)
    leader_vel = rospy.Subscriber(leader_robot_name + "/odom", Odometry, odom_cb)
    listener = tf.TransformListener()

    follower_vel = rospy.Publisher(follower_robot_name + '/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10.0)

    # dynamic_reconfigure
    srv = Server(tf_pidConfig, pid_cb)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(leader_robot_name+'/base_link', follower_robot_name+'/front', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        dis = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        delta_x = trans[0]
        delta_y = trans[1]
        delta_theta = transformations.euler_from_quaternion(rot)[2]

        err_x = L0 * math.cos(theta0) - delta_x
        # 纵向误差
        err_y = L0 * math.sin(theta0) - delta_y

        v_follower = (k_1 * err_x - delta_y * w_leader + v_leader) * math.cos(delta_theta) + (k_2 * err_y + delta_x * w_leader) * math.sin(delta_theta)
        w_follower = ((k_2 * err_y + delta_x * w_leader) * math.cos(delta_theta) - (k_1 * err_x - delta_y * w_leader + v_leader) * math.sin(delta_theta))/d	
	
	
        msg = Twist()
        msg.linear.x = v_follower
        msg.angular.z = w_follower
        follower_vel.publish(msg)

        rate.sleep()
