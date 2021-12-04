#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import actionlib

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class task_2_plan(object):

    def __init__(self):
        super(task_2_plan, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        group_arm = "gripper"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        gripper_group =  moveit_commander.MoveGroupCommander(group_arm)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.gripper_group = gripper_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

    def go_to_joint_state(self, theta):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.gripper_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        # rospy.loginfo(joint_goal)
        joint_goal[0] = theta


        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

    def go_to_pose_goal(self, pose_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group
        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.x -= scale * 0.085  # Second move forward in (x)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    

def main():
    try:
        close_theta = 0.8040252734021681
        open = 0
        delay = 0.1
        input(
            "============ Press `Enter` to begin the simulation by setting up the moveit_commander ..."
        )
        tutorial = task_2_plan()

        pose_home = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(math.pi, 0, math.pi/2, axes='sxyz')
        pose_home.orientation.x = quat[0]
        pose_home.orientation.y = quat[1]
        pose_home.orientation.z = quat[2]
        pose_home.orientation.w = quat[3]
        pose_home.position.x = 0.2017354234423529
        pose_home.position.y = 0.10914157536538296
        pose_home.position.z = 0.8250508366632884

        #=========================================
        ## tomato 1 begin
        #=========================================
        pose_goal_0 = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(math.pi, 0, math.pi, axes='sxyz')
        pose_goal_0.orientation.x = quat[0]
        pose_goal_0.orientation.y = quat[1]
        pose_goal_0.orientation.z = quat[2]
        pose_goal_0.orientation.w = quat[3]
        pose_goal_0.position.x = 0.43272636809439946
        pose_goal_0.position.y = 0.30155827326090934
        pose_goal_0.position.z = 0.7064734758972365
        rospy.sleep(delay)
        tutorial.go_to_pose_goal(pose_goal_0)

        pose_goal_1 = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(math.pi, 0, -math.pi/2, axes='sxyz')
        pose_goal_1.orientation.x = quat[0]
        pose_goal_1.orientation.y = quat[1]
        pose_goal_1.orientation.z = quat[2]
        pose_goal_1.orientation.w = quat[3]
        pose_goal_1.position.x = 0.43272636809439946
        pose_goal_1.position.y = 0.49155827326090934
        pose_goal_1.position.z = 0.7064734758972365
        rospy.sleep(delay)
        tutorial.go_to_pose_goal(pose_goal_1)

        pose_goal_2 = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(math.pi, 0, -math.pi/2, axes='sxyz')
        pose_goal_2.orientation.x = quat[0]
        pose_goal_2.orientation.y = quat[1]
        pose_goal_2.orientation.z = quat[2]
        pose_goal_2.orientation.w = quat[3]

        pose_goal_2.position.x = 0.30346005217719014
        pose_goal_2.position.y = 0.5513129401745189
        pose_goal_2.position.z = 0.6636347917764825
        rospy.sleep(delay)
        tutorial.go_to_pose_goal(pose_goal_2)

        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        # input("============ Press `Enter` to execute a saved path ...")
        rospy.sleep(delay)
        tutorial.execute_plan(cartesian_plan)

        # input("============ Press `Enter` to execute a movement using a predefined goal 2 ...")
        rospy.sleep(delay)
        tutorial.go_to_joint_state(close_theta)
        rospy.sleep(delay)
        tutorial.go_to_pose_goal(pose_goal_2)
        rospy.sleep(delay)
        pose_goal_3 = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(math.pi, 0, 0, axes='sxyz')
        pose_goal_3.orientation.x = quat[0]
        pose_goal_3.orientation.y = quat[1]
        pose_goal_3.orientation.z = quat[2]
        pose_goal_3.orientation.w = quat[3]

        pose_goal_3.position.x = 0.5620792488276463
        pose_goal_3.position.y = 0.548274703294862
        pose_goal_3.position.z = 0.7004179607681765
        rospy.sleep(delay)
        tutorial.go_to_pose_goal(pose_goal_3)
        rospy.sleep(delay)
        tutorial.go_to_joint_state(open)
        #=========================================
        ## tomato 1 done
        #=========================================

        #=========================================
        ## tomato 2 begin
        #=========================================
        tutorial.go_to_pose_goal(pose_goal_1)

        pose_goal_4 = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(math.pi, 0, -math.pi/2, axes='sxyz')
        pose_goal_4.orientation.x = quat[0]
        pose_goal_4.orientation.y = quat[1]
        pose_goal_4.orientation.z = quat[2]
        pose_goal_4.orientation.w = quat[3]
        # x: 0.4795595599174515
        # y: 0.6414064379415384
        # z: 0.9182789912083482
        pose_goal_4.position.x = 0.4795595599174515
        pose_goal_4.position.y = 0.6440064379415384
        pose_goal_4.position.z = 0.9182789912083482
        rospy.sleep(delay)
        tutorial.go_to_pose_goal(pose_goal_4)
        cartesian_plan, fraction = tutorial.plan_cartesian_path()
        rospy.sleep(delay)
        tutorial.execute_plan(cartesian_plan)
        rospy.sleep(delay)
        tutorial.go_to_joint_state(close_theta)
        rospy.sleep(delay)
        tutorial.go_to_pose_goal(pose_goal_4)
        rospy.sleep(delay)
        pose_goal_5 = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(math.pi, 0, 0, axes='sxyz')
        pose_goal_5.orientation.x = quat[0]
        pose_goal_5.orientation.y = quat[1]
        pose_goal_5.orientation.z = quat[2]
        pose_goal_5.orientation.w = quat[3]
        # x: 0.47946054091025203
        # y: 0.3764251813086777
        # z: 0.7083967151435929
        pose_goal_5.position.x = 0.47946054091025203
        pose_goal_5.position.y = 0.502364251813086777
        pose_goal_5.position.z = 0.7583967151435929
        tutorial.go_to_pose_goal(pose_goal_5)
        tutorial.go_to_joint_state(open)
        #=========================================
        ## tomato 2 done
        #=========================================

        #=========================================
        ## tomato 3 begin
        #=========================================
        tutorial.go_to_pose_goal(pose_goal_4)
        pose_goal_6 = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(math.pi, 0, -math.pi/2, axes='sxyz')
        pose_goal_6.orientation.x = quat[0]
        pose_goal_6.orientation.y = quat[1]
        pose_goal_6.orientation.z = quat[2]
        pose_goal_6.orientation.w = quat[3]
        # x: 0.1567514926990433
        # y: 0.4747551398355176
        # z: 1.1936413983356673
        pose_goal_6.position.x = 0.1867514926990433
        pose_goal_6.position.y = 0.4747551398355176
        pose_goal_6.position.z = 1.1936413983356673
        rospy.sleep(delay)
        tutorial.go_to_pose_goal(pose_goal_6)
        cartesian_plan, fraction = tutorial.plan_cartesian_path()
        rospy.sleep(delay)
        tutorial.execute_plan(cartesian_plan)
        rospy.sleep(delay)
        tutorial.go_to_joint_state(close_theta)
        rospy.sleep(delay)
        tutorial.go_to_pose_goal(pose_goal_6)
        rospy.sleep(delay)
        pose_goal_7 = geometry_msgs.msg.Pose()
        quat = quaternion_from_euler(math.pi, 0, 0, axes='sxyz')
        pose_goal_7.orientation.x = quat[0]
        pose_goal_7.orientation.y = quat[1]
        pose_goal_7.orientation.z = quat[2]
        pose_goal_7.orientation.w = quat[3]
        # x: 0.47946054091025203
        # y: 0.3764251813086777
        # z: 0.7083967151435929
        pose_goal_7.position.x = 0.47946054091025203
        pose_goal_7.position.y = 0.502364251813086777
        pose_goal_7.position.z = 0.7583967151435929
        tutorial.go_to_pose_goal(pose_goal_5)
        tutorial.go_to_joint_state(open)
        #=========================================
        ## tomato 3 done
        #=========================================

        print("============ All three tomatoes collected")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()