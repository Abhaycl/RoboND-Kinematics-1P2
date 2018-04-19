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


# Definition of the homogeneous transformation matrix
def HTF_Matrix(alpha, a, d, q):
    HTF = Matrix([[            cos(q),           -sin(q),           0,             a],
                  [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                  [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                  [                 0,                 0,           0,             1]])
    return HTF

# Definition of the functions for the homogeneous transformation matrices of the rotations around x, y, z passing a specific angle
# Rotation (roll)
def Rot_x(r):
    r_x = Matrix([[ 1,      0,       0],
                  [ 0, cos(r), -sin(r)],
                  [ 0, sin(r),  cos(r)]])
    return(r_x)

# Rotation (pitch)
def Rot_y(p):
    r_y = Matrix([[  cos(p), 0, sin(p)],
                  [       0, 1,      0],
                  [ -sin(p), 0, cos(p)]])
    return(r_y)

# Rotation (yaw)
def Rot_z(y):
    r_z = Matrix([[ cos(y), -sin(y), 0],
                  [ sin(y),  cos(y), 0],
                  [      0,       0, 1]])
    return(r_z)


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        # Joint angle variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        # Link displacement variables
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        # Link length variables
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        # Twist angle variables
        alpha0,alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        # r: roll, p: pitch, y: yaw
        r, p, y = symbols('r p y')
        #
        #
        # Create Modified DH parameters
        dh = {alpha0:     0, a0:      0, d1:   0.75, q1:      q1,
              alpha1: -pi/2, a1:   0.35, d2:      0, q2: q2-pi/2,
              alpha2:     0, a2:   1.25, d3:      0, q3:      q3,
              alpha3: -pi/2, a3: 0.0536, d4: 1.5014, q4:      q4,
              alpha4:  pi/2, a4:      0, d5:      0, q5:      q5,
              alpha5: -pi/2, a5:      0, d6:      0, q6:      q6,
              alpha6:     0, a6:      0, d7:  0.303, q7:       0}
        #
        #
        # Define Modified DH Transformation matrix
        #
        # Defined at the beginning
        #
        # Create individual transformation matrices
        HT0_1 = HTF_Matrix(alpha0, a0, d1, q1).subs(dh)
        HT1_2 = HTF_Matrix(alpha1, a1, d1, q2).subs(dh)
        HT2_3 = HTF_Matrix(alpha2, a2, d1, q3).subs(dh)
        HT3_4 = HTF_Matrix(alpha3, a3, d1, q4).subs(dh)
        HT4_5 = HTF_Matrix(alpha4, a4, d1, q5).subs(dh)
        HT5_6 = HTF_Matrix(alpha5, a5, d1, q6).subs(dh)
        HT6_G = HTF_Matrix(alpha6, a6, d1, q7).subs(dh)
        
        # Matrix of the transformations
        HT0_G = HT0_1 * HT1_2 * HT2_3 * HT3_4 * HT4_5 * HT5_6 * HT6_G 
        #
        #
        # Extract rotation matrices from the transformation matrices
        rot_rpy = Rot_x(r) * Rot_y(p) * Rot_z(y)
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
            rot_corr = Rot_z.subs(y, pi) * Rot_y.subs(p, -pi/2)
            rot_rpy = rot_rpy * rot_corr
            rot_rpy = rot_rpy.subs({'r': roll, 'p': pitch, 'y': yaw})
            #
            #
            # Calculate joint angles using Geometric IK method
            d_7 = dh[d7]
            wx = px - (d_7 * rot_rpy[0,2])
            wy = py - (d_7 * rot_rpy[1,2])
            wz = pz - (d_7 * rot_rpy[2,2])
            
            # Leveraging DH distances and offsets
            d_1 = dh[d1]
            d_4 = dh[d4]
            a_1 = dh[a1]
            a_2 = dh[a2]
            a_3 = dh[a3]
            
            # For the evaluation of the angles we apply the law of cosine
            # Calculating theta 1 to 3
            r = sqrt(wx**2 + wy**2) - a_1
            s = wz - d_1
            
            # Use of the cosine law
            s_a = sqrt(a_3**2 + d_4**2)
            s_b = sqrt(s**2 + r**2)
            s_c = a_2
            
            alpha = acos((s_c**2 + s_b**2 - s_a**2) / (2 * s_c * s_b))
            beta = acos((s_c**2 + s_a**2 - s_b**2) / (2 * s_c * s_a))
            
            theta1 = atan2(wy, wx)
            theta2 = (pi/2) - alpha - atan2(s, r)
            theta3 = (pi/2) - beta - atan2(-a_3, d_4)
            
            # Calculating Euler angles from orientation
            # Calculating theta 4 to 6
            R0_3 = HT0_1[0:3, 0:3] * HT1_2[0:3, 0:3] * HT2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3})
            R3_6 = R0_3.HT * rot_rpy
            
            theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
            # Choosing between multiple possible solutions:
            if sin(theta5) < 0:
                theta4 = atan2(-R3_6[2, 2],  R3_6[0, 2])
                theta6 = atan2( R3_6[1, 1], -R3_6[1, 0])
            else:
                theta4 = atan2( R3_6[2, 2], -R3_6[0, 2])
                theta6 = atan2(-R3_6[1, 1],  R3_6[1, 0])
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