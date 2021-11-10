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
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    yaw = euler_from_quaternion([x,y,z,w])[2]
    # rospy.loginfo(math.degrees(yaw))


def laser_callback(msg):
    global regions
    regions = {
        'bright_1':  min(min(msg.ranges[0:3]), msg.range_max) ,
        'bright_2':  min(min(msg.ranges[45:143]), msg.range_max) ,

        'fright_1':  min(min(msg.ranges[144:215]), msg.range_max) ,
        'fright_2':  min(min(msg.ranges[216:287]), msg.range_max) ,

        'front':   min(min(msg.ranges[288:431]), msg.range_max) ,

        'fleft_2':   min(min(msg.ranges[432:503]), msg.range_max) ,
        'fleft_1':   min(min(msg.ranges[504:575]), msg.range_max) ,

        'bleft_2':   min(min(msg.ranges[576:674]), msg.range_max) ,
        'bleft_1':   min(min(msg.ranges[716:719]), msg.range_max) ,
    }
    # rospy.loginfo(regions['bleft_1'] - regions['bright_1'])


def control_loop(publisher):
    rate = rospy.Rate(10)
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    publisher.publish(velocity_msg)
    rate.sleep()
    # while not rospy.is_shutdown():
    home(velocity_msg, publisher)
    lane_travel(0.5, velocity_msg, publisher)
    # rate.sleep()


def move(publisher, speed, vel_msg):    
    vel_msg.linear.x = speed
    publisher.publish(vel_msg)


def rotate(publisher, vel_msg, relative_angle_degree, clockwise):
    kP = 0.8
    error_prior = 0
    integral_prior = 0
    kI = 0.0045
    kD = 0
    t0 = rospy.Time.now().to_sec()
    loop_rate = rospy.Rate(10)
    loop_rate.sleep()    
    while not rospy.is_shutdown():        
        target_rad = math.radians(relative_angle_degree)
        error = target_rad - yaw
        
        t1 = rospy.Time.now().to_sec()
        integral = integral_prior + abs(error) * (t1-t0)
        derivative = (error - error_prior)/(t1-t0)
        if clockwise:
            vel_msg.angular.z = -(kP * abs(error) + kI * integral + kD * derivative)
        else:
            vel_msg.angular.z = kP * abs(error) + kI * integral + kD * derivative
        publisher.publish(vel_msg)
        # rospy.loginfo(target_rad - yaw)
        integral_prior = integral
        error_prior = error
        loop_rate.sleep()
        if abs(error) <= 0.01:
            rospy.loginfo("reached")
            break
    vel_msg.angular.z = 0
    publisher.publish(vel_msg)


def home(vel_msg, publisher):
    rotate(publisher, vel_msg, 180, False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        move(publisher, 0.5, vel_msg)
        rate.sleep()
        if(regions['front'] <= 1.5):
            vel_msg.linear.x = 0
            publisher.publish(vel_msg)
            rospy.loginfo("reached2")
            break
    rotate(publisher, vel_msg, 90, True)


def lane_travel(lin, vel_msg, publisher):
    count = 0
    move(publisher, lin, vel_msg)
    rospy.Rate(1).sleep()
    while not rospy.is_shutdown():
        move(publisher, lin, vel_msg)
        if(regions['bleft_1'] - regions['bright_1'] < 0):
            count = count + 1
            rospy.loginfo(count)
            rospy.Rate(1).sleep()
            if(count == 11):
                rospy.loginfo("left lane exit")
                vel_msg.linear.x = 0
                publisher.publish(vel_msg)
                break


if __name__ == '__main__':
    try:
        rospy.init_node('ebot_controller')
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        laser = rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        odom = rospy.Subscriber('/odom', Odometry, odom_callback)
        control_loop(velocity_publisher)
    except rospy.ROSInterruptException:
        pass
