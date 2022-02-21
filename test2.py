#!/usr/bin/python
# Gazebo UR5
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from math import *
from random import uniform
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
# [0.0, -pi/2, pi/2, pi/3, 0, -pi/10]
# waypoints = [[uniform(-pi, pi) for _ in range(0,6)], [0,0,0,0,0,0]]

def main():

    rospy.init_node('send_joints')
    #pub = rospy.Publisher('/action_move', std_msgs.msg.Int8, queue_size=1)
    pub = rospy.Publisher('/arm_controller/command',
                          JointTrajectory,
                          queue_size=10)


    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    rate = rospy.Rate(1)
    cnt = 0
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():

        pts.positions = [uniform(0,pi),uniform(-0.1111*pi,-pi/2),uniform(-pi/4,pi/4),uniform(0,pi/6),uniform(0,pi),uniform(0,pi)]
        #pts.positions = [0.0, -0.3137*pi, 0.3028*pi, 0.0083*pi, 0.2472*pi, 0.0]
        pts.time_from_start = rospy.Duration(1.0)
        cnt+=1
        cnt%=2
        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        # print(len(pts.positions))
        pub.publish(traj)
        rate.sleep()

    # pts.positions = [0.0, -0.3137*pi, 0.3028*pi, 0.0083*pi, 0.2472*pi, 0.0]
    # pts.time_from_start = rospy.Duration(1.0)
    # cnt+=1
    # cnt%=2
    # # Set the points to the trajectory
    # traj.points = []
    # traj.points.append(pts)
    # # Publish the message
    # # print(len(pts.positions))
    # pub.publish(traj)
    # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Programm interrupted before completion")

