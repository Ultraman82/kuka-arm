#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

            
            # Joint angle symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

      
            # Modified DH params
            s = {
                alpha0:     0, a0:      0, d1: 0.75, # joint 1
                alpha1: -pi/2, a1:   0.35, d2:    0, q2: q2 - pi/2, # joint2
                alpha2:     0, a2:   1.25, d3:    0, # joint3
                alpha3: -pi/2, a3: -0.054, d4: 1.50, # joint4
                alpha4:  pi/2, a4:      0, d5:    0, # joint5
                alpha5: -pi/2, a5:      0, d6:    0, # joint6
                alpha6:     0, a6:      0, d7:0.303, q7: 0 # gripper link
            }

            
            # Define Modified DH Transformation matrix
            print(cos(pi))

            # T0_1 = make_trans_matrix(alpha0, a0, d1, q1).subs(s)
            T0_1 = Matrix([[            cos(q1),           -sin(q1),           0,                 a0],
                           [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                           [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                           [                 0,                 0,           0,                    1]]).subs(s)

            # T1_2 = make_trans_matrix(alpha1, a1, d2, q2).subs(s)
            T1_2 = Matrix([[            cos(q2),           -sin(q2),           0,                 a1],
                           [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                           [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                           [                 0,                 0,           0,                    1]]).subs(s)

            # T2_3 = make_trans_matrix(alpha2, a2, d3, q3).subs(s)
            T2_3 = Matrix([[            cos(q3),           -sin(q3),           0,                 a2],
                           [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                           [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                           [                 0,                 0,           0,                    1]]).subs(s)
            # T3_4 = make_trans_matrix(alpha3, a3, d4, q4).subs(s)
            T3_4 = Matrix([[            cos(q4),           -sin(q4),           0,                 a3],
                           [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                           [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                           [                 0,                 0,           0,                    1]]).subs(s)
            # T4_5 = make_trans_matrix(alpha4, a4, d5, q5).subs(s)
            T4_5 = Matrix([[            cos(q5),           -sin(q5),           0,                 a4],
                           [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                           [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                           [                 0,                 0,           0,                    1]]).subs(s)
            # T5_6 = make_trans_matrix(alpha5, a5, d6, q6).subs(s)
            T5_6 = Matrix([[            cos(q6),           -sin(q6),           0,                 a5],
                           [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                           [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                           [                 0,                 0,           0,                    1]]).subs(s)
            # T6_G = make_trans_matrix(alpha6, a6, d7, q7).subs(s)
            T6_G = Matrix([[            cos(q7),           -sin(q7),           0,                 a6],
                           [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                           [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                           [                 0,                 0,           0,                    1]]).subs(s)
            
            print('T_i_i+1 finish!')

            T0_2 = T0_1 * T1_2
            T0_3 = T0_2 * T2_3
            T0_4 = T0_3 * T3_4
            T0_5 = T0_4 * T4_5
            T0_6 = T0_5 * T5_6
            T0_G = T0_6 * T6_G


            # Create individual transformation matrices
            RC_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                           [sin(pi),  cos(pi), 0, 0],
                           [      0,        0, 1, 0],
                           [      0,        0, 0, 1]])

            RC_y = Matrix([[ cos(-pi/2),  0, sin(-pi/2), 0],
                           [          0,  1,          0, 0],
                           [-sin(-pi/2),  0, cos(-pi/2), 0],
                           [          0,  0,          0, 1]])

            R_corr = RC_z * RC_y

            T_total = T0_G * R_corr

            
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method

            print(px, py, pz, roll, pitch, yaw)

            R_x = Matrix([[1,         0,          0],
                          [0, cos(roll), -sin(roll)],
                          [0, sin(roll),  cos(roll)]])

            R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
                          [          0, 1,          0],
                          [-sin(pitch), 0, cos(pitch)]])

            R_z = Matrix([[cos(yaw), -sin(yaw),  0],
                          [sin(yaw),  cos(yaw),  0],
                          [       0,         0,  1]])

            R0_G = R_x * R_y * R_z  * R_corr[:3,:3].T

            Pwc = Matrix([[px],
                          [py],
                          [pz]]) - s[d7] * R0_G[:,2]
            

            print('Pwc is {}'.format(Pwc))

            """ theta1 """
            theta1 = atan2(Pwc[1,0], Pwc[0,0])
            print('theta 1 is {}'.format(theta1))
            # if theta1 < 0:
            #     theta1 += pi
            
            """ theta3 """
            P0_2 = T0_2.subs({q1: theta1, q2: 0})[:3,3]
            P2_4 = Pwc - P0_2

            l2_4 = sqrt(P2_4[0,0]**2 + P2_4[1, 0]**2 + P2_4[2,0]**2)

            l1 = sqrt(s[a3]**2 +  s[d4]**2)
            s[a2] # is the a2

            def get_triangle(a, b, c):
                v = (a**2 + b**2 - c**2) / (2*a*b)
                return acos(v)

            # v1 = (l1**2 - s[a2]**2 + l2_4**2)/(2*l2_4*l1)
            # print('v1 is {}'.format(v1))
            # v2 = (l2_4 - (l1**2 - s[a2]**2 + l2_4**2)/(2*l2_4) ) / s[a2]
            # print('v2 is {}'.format(v2))
            # phi = asin(v1) + asin(v2)
            phi = get_triangle(l1, s[a2], l2_4)
            print('phi is {}'.format(phi))

            alpha = atan2(s[d4], -s[a3])
            print('alpha is {}'.format(alpha))

            # theta3 = pi - phi - alpha if phi > 0 else pi + phi - alpha
            theta3 = alpha - phi
            # theta3 = theta3 if theta3 > 0 else pi + phi -alpha

            print('theta3 is {}'.format(theta3))

            """ theta2 """
            R0_2 = T0_2.subs({q1: theta1, q2: 0})[:3,:3]
            R2_0 = R0_2.T
            P2_4_2 = R2_0 * P2_4

            beta1 = atan2(P2_4_2[0,0], P2_4_2[1, 0])
            # beta2 = asin((s[a2]**2 - l2_4**2 + l1**2)/(2*l1*s[a2])) \
            #        +asin((l1 - (s[a2]**2 - l2_4**2 + l1**2)/(2*l1))/(l2_4))
            beta2 = get_triangle(s[a2], l2_4, l1)

            print(beta1)
            print(beta2)

            # theta2 = pi/2 - (abs(beta1) + beta2) if beta1 < 0 else pi/2 + (abs(beta1) - beta2)
            theta2 =   pi/2 - (beta2 + beta1)
            # theta2 = theta2 if theta2 < 0 else pi/2 + (abs(beta1) - beta2)


            R0_6 = R0_G #* R_corr[:3,:3].T

            """theta4"""
            R0_4 = T0_4.subs({q1: theta1, q2: theta2, q3: theta3, q4: 0})[:3,:3]

            R4_6 = R0_4.T * R0_6 

            theta4 = atan2(-R4_6[1,2], -R4_6[0,2])
            theta6 = atan2(-R4_6[2,1], R4_6[2,0])


            """theta5"""
            N0_4 = T0_4.subs({q1: theta1, q2: theta2, q3: theta3, q4: theta4})[:3,2]
            Z0_5 = T0_5.subs({q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: 0})[:3,2]

            N0_6 = R0_6[:,2]
            x0406 = N0_4.cross(N0_6)
            print('x0406 is {}'.format(x0406))
            theta5 = asin(Z0_5.dot(x0406))

            print('theta5 is {}'.format(theta5))

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
