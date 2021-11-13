#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

x = 0
y = 0
yaw = 0

def laser_callback(msg):
    global regions
    regions = {
        'bright_1':  min(min(msg.ranges[0:3]), msg.range_max) ,
        'fright':  min(min(msg.ranges[144:287]),msg.range_max) ,
        'front':   min(min(msg.ranges[288:431]),msg.range_max) ,
        'fleft':   min(min(msg.ranges[432:575]),msg.range_max) ,
        'bleft_1':   min(min(msg.ranges[716:719]), msg.range_max) ,
    }
    rospy.loginfo(regions['bleft_1'] - regions['bright_1'])

def odom_callback(data):
    # global pose
    global x, y, yaw
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    # z = data.pose.pose.orientation.z;
    yaw = data.pose.pose.orientation.w;
    # pose = {
    #     'x':    data.pose.pose.position.x,
    #     'y':    data.pose.pose.position.y,
    #     'yaw':  euler_from_quaternion([x,y,z,w])[2]*180/math.pi
    # }
    # print(pose)
        
# def control_loop():
    # rospy.init_node('ebot_controller')
    
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    # rospy.Subscriber('/odom', Odometry, odom_callback)
    
    # rate = rospy.Rate(10) 

    # velocity_msg = Twist()
    # velocity_msg.linear.x = 0
    # velocity_msg.angular.z = 0
    # pub.publish(velocity_msg)

    # while not rospy.is_shutdown():
        
    #     #

    #     # Your algorithm to complete the obstacle course
    #     #

    #     # velocity_msg.linear.x = 
    #     # velocity_msg.angular.z = 
    #     # pub.publish(velocity_msg)
    #     home(-0.8651786644148678, -0.7284927599560349, pub)
    #     print("Controller message pushed at {}".format(rospy.get_time()))
    #     rate.sleep()


# def lane_travel():

def home(x_goal, y_goal, velocity_publisher):
    global x
    global y, yaw
    # global pose
    velocity_message = Twist()
    # cmd_vel_topic='/turtle1/cmd_vel'
    # rate = rospy.Rate(10)
    while (True):
        rate.sleep()
        K_linear = 0.1
        distance = abs(math.sqrt(((x_goal - x) ** 2) + ((y_goal - y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 0.6
        desired_angle_goal = math.atan2(y_goal- y, x_goal - x)
        angular_speed = (desired_angle_goal - yaw) * K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        
        velocity_publisher.publish(velocity_message)

        if (distance < 0.01):
            break


if __name__ == '__main__':
    try:
        rospy.init_node('ebot_controller')
    
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        rospy.Subscriber('/odom', Odometry, odom_callback)
        
         

        velocity_msg = Twist()
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        pub.publish(velocity_msg)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            #

            # Your algorithm to complete the obstacle course
            #

            # velocity_msg.linear.x = 
            # velocity_msg.angular.z = 
            # pub.publish(velocity_msg)
            # home(-0.8651, -0.72, pub)
            # print("Controller message pushed at {}".format(rospy.get_time()))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
