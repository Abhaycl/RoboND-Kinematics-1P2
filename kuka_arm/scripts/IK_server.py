#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


# Define Modified DH Transformation matrix
def HTF_Matrix(joint, twist, length, offset):
    HTF = Matrix([[            cos(joint),            -sin(joint),           0,             length],
                  [ sin(joint)*cos(twist), cos(joint1)*cos(twist), -sin(twist), -sin(twist)*offset],
                  [ sin(joint)*sin(twist), cos(joint1)*sin(twist),  cos(twist),  cos(twist)*offset],
                  [                     0,                      0,           0,                  1]])
    return HTF


# Rotation (roll)
def Rot_x(r):
    R_x = Matrix([[ 1,      0,       0],
                  [ 0, cos(r), -sin(r)],
                  [ 0, sin(r),  cos(r)]])
    return(R_x)


# Rotation (pitch)
def Rot_y(p):
    R_y = Matrix([[  cos(p), 0, sin(p)],
                  [       0, 1,      0],
                  [ -sin(p), 0, cos(p)]])
    return(R_y)


# Rotation (yaw)
def Rot_z(y):
    R_z = Matrix([[ cos(y), -sin(y), 0],
                  [ sin(y),  cos(y), 0],
                  [      0,       0, 1]])
    return(R_z)


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        # Representative variables of joint angles
        joint1, joint2, joint3, joint4, joint5, joint6, joint7 = symbols('joint1:8')
        # Link displacement variables
        offset1, offset2, offset3, offset4, offset5, offset6, offset7 = symbols('offset1:8')
        # Link length variables
        length0, length1, length2, length3, length4, length5, length6 = symbols('length0:7')
        # Rotation angle variables
        twist0, twist1, twist2, twist3, twist4, twist5, twist6 = symbols('twist0:7')
        # r: roll, p: pitch, y: yaw
        r, p, y = symbols('r p y')
        #
        #
        # Create Modified DH parameters
        dh = {twist0:     0, length0:      0, offset1:  0.75,
              twist1: -pi/2, length1:   0.35, offset2:     0, joint2: joint2-pi/2,
              twist2:     0, length2:   1.25, offset3:     0,
              twist3: -pi/2, length3: -0.054, offset4:   1.5,
              twist4:  pi/2, length4:      0, offset5:     0,
              twist5: -pi/2, length5:      0, offset6:     0,
              twist6:     0, length6:      0, offset7: 0.303, joint7:           0}
        #
        #
        # Define Modified DH Transformation matrix
        #
        #
        # Create individual transformation matrices
        HT0_1 = HTF_Matrix(joint1, twist0, length0, offset1).subs(dh)
        HT1_2 = HTF_Matrix(joint2, twist1, length1, offset2).subs(dh)
        HT2_3 = HTF_Matrix(joint3, twist2, length2, offset3).subs(dh)
        HT3_4 = HTF_Matrix(joint4, twist3, length3, offset4).subs(dh)
        HT4_5 = HTF_Matrix(joint5, twist4, length4, offset5).subs(dh)
        HT5_6 = HTF_Matrix(joint6, twist5, length5, offset6).subs(dh)
        HT6_G = HTF_Matrix(joint7, twist6, length6, offset7).subs(dh)
        
        # Matrix of the transformations
        HT0_G = HT0_1 * HT1_2 * HT2_3 * HT3_4 * HT4_5 * HT5_6 * HT6_G 
        #
        #
        # Extract rotation matrices from the transformation matrices
        R_corr = R_z(pi) * R_y(-pi/2)
        R_rpy = R_z(y) * R_y(p) * R_x(r)
        #
        #
        ###
        
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            
            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rot_rpy = Rot_rpy.evalf(subs={r: roll, p: pitch, y: yaw})
            R0_6 = Rot_rpy * Rot_corr
            #
            #
            # Calculate joint angles using Geometric IK method
            d_7 = dh[offset7]
            wx = pos_x - (d_7 * R0_6[0,2])
            wy = pos_y - (d_7 * R0_6[1,2])
            wz = pos_z - (d_7 * R0_6[2,2])
            
            # Leveraging DH distances and offsets
            d_1 = dh[offset1]
            d_4 = dh[offset4]
            a_1 = dh[length1]
            a_2 = dh[length2]
            a_3 = dh[length3]
            
            # For the evaluation of the angles we apply the law of cosine
            # Calculating theta 1 to 3
            r = sqrt(wx**2 + wy**2) - a_1
            s = wz - d_1
            
            s_a = sqrt(a_3**2 + d_4**2)
            s_b = sqrt(s**2 + r**2)
            s_c = a_2
            
            alpha = acos((s_c**2 + s_b**2 - s_a**2) / (2 * s_c * s_b))
            beta = acos((s_c**2 + s_a**2 - s_b**2) / (2 * s_c * s_a))
            
            theta1 = atan2(wy, wx)
            theta2 = (pi/2) - alpha - atan2(s, r)
            theta3 = (pi/2) - beta - atan2(-a_3, d_4)
            
            # Calculating theta 4 to 6
            R0_3 = HT0_1 * HT1_2 * HT2_3
            R0_3 = R0_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={joint1: theta1, joint2: theta2, joint3: theta3})
            R3_6 = R0_3.inv("ADJ") * R0_6 
            
            r12 = R3_6[0, 1]
            r13 = R3_6[0, 2]
            r21 = R3_6[1, 0]
            r22 = R3_6[1, 1]
            r23 = R3_6[1, 2]
            r32 = R3_6[2, 1]
            r33 = R3_6[2, 2]
            
            theta4 = atan2(r33, -r13)
            theta5 = atan2(sqrt(r13**2 + r33**2), r23)
            theta6 = atan2(-r22, r21)
            #
            #
            ###
            
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)
        
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()