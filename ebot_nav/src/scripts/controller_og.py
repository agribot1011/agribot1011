#!/usr/bin/env python3
#note: (bleft - bright) can tell us which specific lane are we exiting
# +ve: rightlane, -ve: leftlane, zero: middlelane
#it can also tell us which lane are we about to enter

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

regions = {}
yaw = 0

def odom_callback(data):
    global yaw
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    yaw = euler_from_quaternion([x,y,z,w])[2]
    # rospy.loginfo(math.degrees(yaw))


def laser_callback(msg):
    global regions
    regions = {
        'bright':  min(min(msg.ranges[0:143]),msg.range_max) ,
        'fright':  min(min(msg.ranges[144:287]),msg.range_max) ,
        'front':   min(min(msg.ranges[288:431]),msg.range_max) ,
        'fleft':   min(min(msg.ranges[432:575]),msg.range_max) ,
        'bleft':   min(min(msg.ranges[576:719]),msg.range_max) ,
    }
    # rospy.loginfo(regions['front'])


def control_loop(publisher):
    rate = rospy.Rate(20)
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    publisher.publish(velocity_msg)
    rate.sleep()
    while not rospy.is_shutdown():
        home(velocity_msg, publisher)
        lane_travel(0.75, velocity_msg, publisher)
        rate.sleep()


def move(publisher, speed, vel_msg):    
    vel_msg.linear.x = speed
    publisher.publish(vel_msg)


def rotate(publisher, vel_msg, relative_angle_degree, clockwise):
    kP = 1.15
    loop_rate = rospy.Rate(10)    
    while not rospy.is_shutdown():
        target_rad = math.radians(relative_angle_degree)
        if clockwise:
            vel_msg.angular.z = -kP * abs(target_rad - yaw)
        else:
            vel_msg.angular.z = kP * abs(target_rad - yaw)

        publisher.publish(vel_msg)
        rospy.loginfo(target_rad - yaw)
        loop_rate.sleep()
        if abs(target_rad - yaw) <= 0.08:
            rospy.loginfo("reached")
            break
    vel_msg.angular.z = 0
    publisher.publish(vel_msg)


def home(vel_msg, publisher):
    rotate(publisher, vel_msg, 180, False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        move(publisher, 0.3, vel_msg)
        rate.sleep()
        if(regions['front'] <= 1.5):
            vel_msg.linear.x = 0
            publisher.publish(vel_msg)
            rospy.loginfo("reached2")
            break
    rotate(publisher, vel_msg, 90, True)


def lane_travel(lin, vel_msg, publisher):
    while not rospy.is_shutdown():
        move(publisher, lin, vel_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ebot_controller')
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        laser = rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        odom = rospy.Subscriber('/odom', Odometry, odom_callback)
        control_loop(velocity_publisher)
    except rospy.ROSInterruptException:
        pass
