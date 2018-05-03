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

## Definition of the homogeneous transformation matrix
#def HTF_Matrix(alpha, a, d, q):
#    HTF = Matrix([[            cos(q),           -sin(q),           0,             a],
#                  [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
#                  [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
#                  [                 0,                 0,           0,             1]])
#    return HTF

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
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        # r: roll, p: pitch, y: yaw
        r, p, y = symbols('r p y')
        #
        #
        # Create Modified DH parameters
        # KUKA KR210
        dh = {alpha0:     0, a0:      0, d1:  0.75, q1:      q1,
              alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
              alpha2:     0, a2:   1.25, d3:     0, q3:      q3,
              alpha3: -pi/2, a3: -0.054, d4:  1.50, q4:      q4,
              alpha4:  pi/2, a4:      0, d5:     0, q5:      q5,
              alpha5: -pi/2, a5:      0, d6:     0, q6:      q6,
              alpha6:     0, a6:      0, d7: 0.303, q7:       0}
        #
        #
        # Define Modified DH Transformation matrix
        #
        # Defined at the beginning
        #
        # Create individual transformation matrices
        # Base_link to Link1
        HT0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                        [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                        [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                        [                  0,                    0,            0,               1]])
        HT0_1 = HT0_1.subs(dh)
        
        # Link1 to Link2
        HT1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                        [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                        [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                        [                  0,                    0,            0,               1]])
        HT1_2 = HT1_2.subs(dh)
        
        # Link2 to Link3
        HT2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                        [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                        [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                        [                  0,                    0,            0,               1]])
        HT2_3 = HT2_3.subs(dh)
        
        # Link3 to Link4
        HT3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                        [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                        [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                        [                  0,                    0,            0,               1]])
        HT3_4 = HT3_4.subs(dh)
        
        # Link4 to Link5
        HT4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                        [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                        [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                        [                  0,                    0,            0,               1]])
        HT4_5 = HT4_5.subs(dh)
        
        # Link5 to Link6
        HT5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                        [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                        [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                        [                  0,                    0,            0,               1]])
        HT5_6 = HT5_6.subs(dh)
        
        # Link6 to Gripper_link
        HT6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                        [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                        [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                        [                  0,                    0,            0,               1]])
        HT6_G = HT6_G.subs(dh)
        
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
            # Leveraging DH distances and offsets
            d_1 = dh[d1] # d1 = 0.75
            d_4 = dh[d4] # d4 = 1.5
            d_7 = dh[d7] # d7 = 0.303
            a_1 = dh[a1] # a1 = 0.35
            a_2 = dh[a2] # a2 = 1.25
            a_3 = dh[a3] # a3 = -0.054
            
            # Calculate joint angles using Geometric IK method
            wx = px - (d_7 * rot_rpy[0,2])
            wy = py - (d_7 * rot_rpy[1,2])
            wz = pz - (d_7 * rot_rpy[2,2])
            
            # Calculating theta 1
            theta1 = atan2(wy, wx)
            
            # For the evaluation of the angles we apply the law of cosine
            # Calculating radius
            r = sqrt(wx**2 + wy**2) - a_1
            
            # Use of the cosine law to calculate theta2 theta3 using A, B, C sides of the triangle
            A = sqrt(a_3**2 + d_4**2) # A = 1.50097
            B = sqrt((wz - d_1)**2 + r**2)
            C = a_2 # C = 1.25
            
            # Calculating theta 2
            alpha = acos((C**2 + B**2 - A**2) / (2 * C * B))
            theta2 = (pi/2) - alpha - atan2((wz - d_1), r)
            # Calculating theta 3
            beta = acos((C**2 + A**2 - B**2) / (2 * C * A))
            theta3 = (pi/2) - beta - atan2(-a_3, d_4)
            
            # Calculating Euler angles from orientation
            R0_3 = HT0_1[0:3, 0:3] * HT1_2[0:3, 0:3] * HT2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3})
            # R3_6 = R0_3.inv("LU")*Rrpy # Calculate inverse of R0_3:
            R3_6 = R0_3.HT * rot_rpy
            
            # Calculating theta 4
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            # Calculating theta 5
            theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
            # Calculating theta 6
            
            #theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
            ## Choosing between multiple possible solutions:
            #if sin(theta5) < 0:
            #    theta4 = atan2(-R3_6[2, 2],  R3_6[0, 2])
            #    theta6 = atan2( R3_6[1, 1], -R3_6[1, 0])
            #else:
            #    theta4 = atan2( R3_6[2, 2], -R3_6[0, 2])
            #    theta6 = atan2(-R3_6[1, 1],  R3_6[1, 0])
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