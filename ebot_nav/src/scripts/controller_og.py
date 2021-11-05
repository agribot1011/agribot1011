#!/usr/bin/env python3
#hello

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

        #
        # Your algorithm to complete the obstacle course
        #

        home(velocity_msg, publisher)
        lane_travel(0.75, velocity_msg, publisher)
        # rate.sleep()


def move(publisher, speed, vel_msg):    
    vel_msg.linear.x = speed
    publisher.publish(vel_msg)


def rotate(publisher, vel_msg, angular_speed_degree, relative_angle_degree, clockwise):
    angular_speed = math.radians(abs(angular_speed_degree))  
    if clockwise:
        vel_msg.angular.z = -1 * abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    loop_rate = rospy.Rate(10)    
    t0 = rospy.Time.now().to_sec()
    yaw0 = yaw
    while not rospy.is_shutdown():
        # rospy.loginfo("Turtlesim rotates")
        publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        desired_angle_degree = (t1 - t0) * angular_speed_degree
        current_angle_degree = math.degrees(yaw - yaw0) 
        error = desired_angle_degree - current_angle_degree
        rospy.loginfo(error)
        loop_rate.sleep()     

        if desired_angle_degree >= relative_angle_degree:
            rospy.loginfo("reached")
            break

    #finally, stop the robot when the distance is moved
    vel_msg.angular.z = 0
    publisher.publish(vel_msg)


def home(vel_msg, publisher):
    rotate(publisher, vel_msg, 30, 178, False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        move(publisher, 0.3, vel_msg)
        rate.sleep()
        if(regions['front'] <= 1.5):
            vel_msg.linear.x = 0
            publisher.publish(vel_msg)
            rospy.loginfo("reached2")
            break
    rotate(publisher, vel_msg, 30, 90, True)


def lane_travel(lin, vel_msg, publisher):
    # global regions
    # edge = regions['fleft']
    # edge = edge - regions['fleft']
    vel_msg.linear.x = lin
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():
        publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ebot_controller')
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        laser = rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        odom = rospy.Subscriber('/odom', Odometry, odom_callback)
        control_loop(velocity_publisher)
    except rospy.ROSInterruptException:
        pass
