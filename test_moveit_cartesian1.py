#!/usr/bin/env python
# -*- coding: utf-8 -*-

from select import POLLWRBAND
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy

import math
import numpy

class MoveItCartesianDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize ros node
        rospy.init_node('moveit_cartesian_test1', anonymous=True)

                
        # initialize move group commander
        arm = MoveGroupCommander('manipulator')

        # allow replanning when failing
        arm.allow_replanning(True)

        # set base_link as the reference koordinate system
        reference_frame = 'base_link'#'base_link'
        arm.set_pose_reference_frame(reference_frame)
        
        # set position(m) and orientation(rad) tolerance
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)

        # max acceleration and velocity
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # get ee_link's name
        end_effector_link = arm.get_end_effector_link()

        # set the roboter arm to 'home' position
        arm.set_named_target('up')
        arm.go()
        rospy.sleep(1)

        #setup the target pose with cartesian values
                                    
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.660262 #np.random.uniform(down,up)
        target_pose.pose.position.y = 0.109284
        target_pose.pose.position.z = 0.439211
        # target_pose.pose.orientation.x = -1.546266
        # target_pose.pose.orientation.y = -0.063824
        # target_pose.pose.orientation.z = -1.559499
        target_pose.pose.orientation.w = 1
    
        
        # set the target pose for ee_link & go!
        arm.set_pose_target(target_pose, end_effector_link)
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
   
        wpose.orientation.x += 1.0
        wpose.orientation.y += 0.5
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
        plan_lib = []

        # set the arm to start state
        arm.set_start_state_to_current_state()

        # using compute_cartesian_path method to plan the path
        while fraction < 1.0 and attempts < maxtries:
        #while attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                            waypoints,   # waypoint poses
                            0.01,        # eef_step
                            0.0,         # jump_threshold
                            True)        # avoid_collision
            

            # attempts documented
            attempts += 1
            
            # print process
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                            
            # if the plan succeeds, execute it and send messsages
            if fraction == 1.0:
                plan_lib.append(plan)
                rospy.loginfo("Path computed successfully. Moving the arm.")
                arm.execute(plan,wait=True)
                rospy.loginfo("Path execution complete.")

                end_pose = arm.get_current_pose(end_effector_link).pose
                print('End Pose: \n', end_pose)
                
                
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
        

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        print ("Programm interrupted before completion")
        
