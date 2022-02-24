#!/usr/bin/env python
# -*- coding: utf-8 -*-

import imp
from select import POLLWRBAND
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_msgs.msg
import sensor_msgs.msg
import shape_msgs.msg
import visualization_msgs.msg
import std_msgs.msg


#import math
from math import *
import numpy


COLOR_RED = std_msgs.msg.ColorRGBA(1.0, 0.0, 0.0, 1.0)
COLOR_GREEN = std_msgs.msg.ColorRGBA(0.0, 1.0, 0.0, 1.0)
COLOR_TRANSLUCENT = std_msgs.msg.ColorRGBA(0.0, 0.0, 0.0, 0.5)

class MoveItCartesianDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize ros node
        rospy.init_node('moveit_cartesian_test1', anonymous=True)

        # initialize move group commander, scene planning interface
        arm = MoveGroupCommander('manipulator')
        scene = PlanningSceneInterface()

        #Create a publisher to visualize the position constraints in Rviz
        marker_publisher = rospy.Publisher(
                    "/visualization_marker", 
                    visualization_msgs.msg.Marker, 
                    queue_size=20,
            )
        rospy.sleep(0.5)  # publisher needs some time to connect Rviz
        #self.remove_all_markers()
        
        # allow replanning when failing
        arm.allow_replanning(True)

        # set base_link as the reference koordinate system
        reference_frame = 'base_link'#'base_link'
        arm.set_pose_reference_frame(reference_frame)
        
        # set position(m) and orientation(rad) tolerance
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)

        # max acceleration and velocity
        arm.set_max_acceleration_scaling_factor(0.25)
        arm.set_max_velocity_scaling_factor(0.25)

        # get ee_link's name
        end_effector_link = arm.get_end_effector_link()

        # set the roboter arm to 'home' position
        arm.set_named_target('up')
        arm.go()
        rospy.sleep(3)
        state = arm.get_current_joint_values()
        print(state)
            
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.50262 #np.random.uniform(down,up)
        target_pose.pose.position.y = 0.109284
        target_pose.pose.position.z = 0.4000

        roll = 0 #y axis
        yaw = 0 #z axis
        pitch = 0 #x axis
        ox, oy, oz = pitch*pi/360, roll*pi/360, yaw*pi/360
        q = quaternion_from_euler(ox,oy,oz)

        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
    
        #Joint constraint
        path_constraints = moveit_msgs.msg.Constraints()
        joint_constraint = moveit_msgs.msg.JointConstraint()

        joint_names = arm.get_joints()
        current_joints = arm.get_current_joint_values()
        print(joint_names)

        ## shoulder_lift_joint
        joint_constraint.joint_name = joint_names[1] # 0,1,2,3,4,5
        joint_constraint.position = current_joints[1] #-90*pi/360
        joint_constraint.tolerance_above = 60*pi/360 #angle constraint
        joint_constraint.tolerance_below = 30*pi/360 #angle constraint
        joint_constraint.weight = 1.0

        path_constraints.joint_constraints.append(joint_constraint)

        #elbow_joint
        # joint_constraint.joint_name = arm.get_joints()[2] # 0,1,2,3,4,5
        # joint_constraint.position = 0.0
        # joint_constraint.tolerance_above = 90*pi/360 #angle constraint
        # joint_constraint.tolerance_below = 0.0*pi/360 #angle constraint
        # joint_constraint.weight = 1.0

        # path_constraints.joint_constraints.append(joint_constraint)
        #setup the target pose with cartesian values
        
        
        # set the target pose for ee_link & go!
        arm.set_pose_target(target_pose, end_effector_link)
        arm.set_path_constraints(path_constraints)

        traj = arm.plan()[1]
        arm.execute(traj, wait=True)
        arm.clear_pose_targets()
        rospy.sleep(1)

        # init waypoints
        waypoints = []
        start_pose = arm.get_current_pose(end_effector_link).pose

        ## DO NOT add start point into the waypoints!
        ##otherwise u will get waypoints not strictly increasing ERROR.
        #start_point = deepcopy(start_pose)
        #waypoints.append(start_point)

        wpose = deepcopy(start_pose)
        
        wpose.position.z -= 0.05
        wpose.position.x -= 0.05
        wpose.position.y += 0.05
        waypoints.append(deepcopy(wpose))


        wpose.position.x -= 0.1
        wpose.position.y += 0.02
        wpose.position.z -= 0.03
        waypoints.append(deepcopy(wpose))

        roll = 90 #y axis
        yaw = 0 #z axis
        pitch = 0 #x axis
        ox, oy, oz = pitch*pi/360, roll*pi/360, yaw*pi/360

        q = quaternion_from_euler(ox,oy,oz)
   
        wpose.orientation.x += q[0]
        wpose.orientation.y += q[1]
        wpose.orientation.z += q[2]
        wpose.orientation.w += q[3]
        waypoints.append(deepcopy(wpose))


        # add the target poses to the waypoints (allow multiple target poses planned)
        # waypoints.append(target_pose.pose)

        # centerA = target_pose.pose.position.y
        # centerB = target_pose.pose.position.z
        # radius = 0.001

        # for th in numpy.arange(0, 6.28, 0.2):
        #     target_pose.pose.position.y = centerA + 0.5*radius * math.cos(th)
        #     target_pose.pose.position.z = centerB + 1.2*radius * math.sin(th)
        #     wpose = deepcopy(target_pose.pose)
        #     waypoints.append(deepcopy(wpose))

       
        fraction = 0.0   #path planned cover-percentage
        maxtries = 50   #max try-times
        attempts = 0     


        # set the arm to start state
        arm.set_start_state_to_current_state()

        # using compute_cartesian_path method to plan the path
        while fraction < 1.0 and attempts < maxtries:
        #while attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                            waypoints,   # waypoint poses
                            0.001,        # eef_step
                            0.0,         # jump_threshold
                            True)        # avoid_collision
            

            # attempts documented
            attempts += 1
            
            # print process
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                            
            # if the plan succeeds, execute it and send messsages
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                arm.execute(plan,wait=True)
                rospy.loginfo("Path execution complete.")

                end_pose = arm.get_current_pose(end_effector_link).pose
                print('End Pose: \n', end_pose)
                
                rospy.sleep(5)

                arm.set_named_target('up')
                arm.clear_path_constraints()
                #arm.plan()
                arm.go()
                arm.clear_pose_targets()
                rospy.sleep(2)
            # if the plan fails, send failure message
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(attempts) + " attempts.")  
                rospy.sleep(1)


            # move the arm back to 'home'
            # arm.set_named_target('home')
            # arm.go()
            rospy.sleep(1)

            # turn off and exit moveit
            # moveit_commander.roscpp_shutdown()
            # moveit_commander.os._exit(0)
        
        from std_srvs.srv import Empty
        rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()


if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        print ("Programm interrupted before completion")
        
