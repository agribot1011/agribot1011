#!/usr/bin/env python3

#note: (bleft - bright) can tell us which specific lane are we exiting
# +ve: rightlane, -ve: leftlane, zero: middlelane
#it can also tell us which lane are we about to enter
#this algorithm only needs 3 samples of the laser scan

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

regions = {}
yaw = 0
current_lane = {  
    'left_lane' : False ,              
    'middle_lane' : False,
    'right_lane' :  False, 
}

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
        'bright_1': min(min(msg.ranges[0:3]), msg.range_max) ,
        # 'bright_2':  min(min(msg.ranges[45:143]), msg.range_max) ,
        # 'fright_1':  min(min(msg.ranges[144:215]), msg.range_max) ,
        # 'fright_2':  min(min(msg.ranges[216:287]), msg.range_max) ,
        'front': msg.ranges[359],
        # 'fleft_2':   min(min(msg.ranges[432:503]), msg.range_max) ,
        # 'fleft_1':   min(min(msg.ranges[504:575]), msg.range_max) ,
        # 'bleft_2':   min(min(msg.ranges[576:674]), msg.range_max) ,
        'bleft_1': min(min(msg.ranges[716:719]), msg.range_max)
    }
    # rospy.loginfo(regions['front'])

#controller function to command the bot
def control_loop(publisher):
    rate = rospy.Rate(10)
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    publisher.publish(velocity_msg)
    rate.sleep()
    home(velocity_msg, publisher)
    lane_travel(0.5, velocity_msg, publisher, current_lane)
    lane_switch(0.2, velocity_msg, publisher, current_lane)
    lane_travel(0.5, velocity_msg, publisher, current_lane)
    lane_switch(0.2, velocity_msg, publisher, current_lane)
    lane_travel(0.5, velocity_msg, publisher, current_lane)

#function to move the ebot in straight direction.
def move(publisher, speed, vel_msg):    
    vel_msg.linear.x = speed
    yaw_error = math.pi/2 - yaw
    # vel_msg.angular.z = 0.01 * yaw_error
    # rospy.loginfo(yaw_error)
    publisher.publish(vel_msg)

# For rotating the robot PID controller has been used. 
# kP = Proportional gain, kI = 	Integral gain, kD = Differential gain 
# t0 = initial time 
def rotate(publisher, vel_msg, target_yaw, clockwise):
    kP = 0.8 
    error_prior = 0
    integral_prior = 0
    kI = 0.0045
    kD = 0
    t0 = rospy.Time.now().to_sec()
    loop_rate = rospy.Rate(10)
    loop_rate.sleep()    
    while not rospy.is_shutdown():        
        target_rad = math.radians(target_yaw)
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
        if abs(error) <= 0.02225:
            rospy.loginfo("reached")
            break
    vel_msg.angular.z = 0
    publisher.publish(vel_msg)

#function to bring the ebot to the left lane
def home(vel_msg, publisher):
    global current_lane
    current_lane['left_lane'] = True        #because we start from left lane
    rotate(publisher, vel_msg, 180, False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        move(publisher, 0.5, vel_msg)
        rate.sleep()

        #stop the bot when it is in the middle
        if(regions['front'] <= 1.5):
            vel_msg.linear.x = 0
            publisher.publish(vel_msg)
            rospy.loginfo("reached2")
            break
    rotate(publisher, vel_msg, 90, True)

#function to handle lane travelling and checking if the lane has been covered
def lane_travel(lin, vel_msg, publisher, presentLane):
    global current_lane

    #this keeps track of how many pots the ebot has covered while traversing the lane
    #there are a total of 10 pots in a single lane
    pots_covered = 0
    
    move(publisher, lin, vel_msg)
    rospy.Rate(0.75).sleep()

    if(presentLane['left_lane']):
        while not rospy.is_shutdown():
            move(publisher, lin, vel_msg)

            #there is some gap present between each pot and we can use the laser scan data to detect this gap
            #when a gap is detected that means the ebot has covered the pot
            if(regions['bleft_1'] - regions['bright_1'] < 0):
                pots_covered = pots_covered + 1
                rospy.loginfo(pots_covered)
                rospy.Rate(1).sleep()

                #stop the bot when it exits the lane
                if(pots_covered == 11):
                    rospy.loginfo("left lane exit")
                    vel_msg.linear.x = 0
                    publisher.publish(vel_msg)
                     
                    #set current lane accordingly
                    current_lane['left_lane'] = False
                    current_lane['middle_lane'] = True
                    break

    elif(presentLane['middle_lane']):
        while not rospy.is_shutdown():
            move(publisher, lin, vel_msg)
            if((regions['bleft_1'] - regions['bright_1'] < 1) and (regions['bleft_1'] > 1 or regions['bright_1'] > 1)):
                pots_covered = pots_covered + 1
                rospy.loginfo(pots_covered)
                rospy.Rate(1).sleep()
                if(pots_covered == 11):
                    rospy.loginfo("middle lane exit")
                    vel_msg.linear.x = 0
                    publisher.publish(vel_msg)
                    current_lane['middle_lane'] = False
                    current_lane['right_lane'] = True
                    break

    elif(presentLane['right_lane']):
        while not rospy.is_shutdown():
            move(publisher, lin, vel_msg)
            if(regions['bleft_1'] - regions['bright_1'] > 0):
                pots_covered = pots_covered + 1
                rospy.loginfo(pots_covered)
                rospy.Rate(1).sleep()
                if(pots_covered == 11):
                    rospy.loginfo("right lane exit")
                    vel_msg.linear.x = 0
                    publisher.publish(vel_msg)
                    break

#function to handle lane switching
#depending upon which lanes to switch we will have 2 cases:
#Case1: FROM left lane to middle lane : we will move the bot then rotate it to 0 degrees clockwise
#       and then again move forward till front is less than 3.5. Now at this position we 
#       can rotate the bot to -90 degrees clockwise to face the middle lane.
#Case2: FROM middle lane to right lane : we will move the bot then rotate it to 0 degrees anticlockwise
#       and then anagin move forward till front is less than 1.5. Now at this position we 
#       can rotate the bot to 90 degrees anticlockwise to face the right lane.
#                                                                -sairaj
def lane_switch(lin, vel_msg, publisher, presentLane):
    if(presentLane['middle_lane']):
        move(publisher, lin, vel_msg)
        rotate(publisher, vel_msg, 0, True)
        while not rospy.is_shutdown():
            move(publisher, lin, vel_msg)
            # rospy.Rate(10).sleep()
            if(regions['front'] < 3.15):
                vel_msg.linear.x = 0
                publisher.publish(vel_msg)
                break
        rotate(publisher, vel_msg, -90, True)

    elif(presentLane['right_lane']):
        move(publisher, lin, vel_msg)
        rotate(publisher, vel_msg, 0, False)
        while not rospy.is_shutdown():
            move(publisher, lin, vel_msg)
            # rospy.Rate(10).sleep()
            if(regions['front'] < 1.25):
                vel_msg.linear.x = 0
                publisher.publish(vel_msg)
                break
        rotate(publisher, vel_msg, 90, False)



if __name__ == '__main__':
    try:
        rospy.init_node('ebot_controller')
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        laser = rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        odom = rospy.Subscriber('/odom', Odometry, odom_callback)
        control_loop(velocity_publisher)
    except rospy.ROSInterruptException:
        pass
