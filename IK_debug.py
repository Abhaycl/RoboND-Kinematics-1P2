from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]
    
    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])
    
    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation
    
    comb = Combine(position,orientation)
    
    class Pose:
        def __init__(self,comb):
            self.poses = [comb]
    
    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ##
    ## Insert IK code here!
    #theta1 = 0
    #theta2 = 0
    #theta3 = 0
    #theta4 = 0
    #theta5 = 0
    #theta6 = 0
    
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
    
    # DH parameterization for the KUKA KR210
    dh = {twist0:     0, length0:      0, offset1:  0.75,
          twist1: -pi/2, length1:   0.35, offset2:     0, joint2: joint2-pi/2,
          twist2:     0, length2:   1.25, offset3:     0,
          twist3: -pi/2, length3: -0.054, offset4:   1.5,
          twist4:  pi/2, length4:      0, offset5:     0,
          twist5: -pi/2, length5:      0, offset6:     0,
          twist6:     0, length6:      0, offset7: 0.303, joint7:           0}
    
    # Definition of functions for homogeneous transformations
    def HTF_Matrix(twist, length, offset, joint):
        HTF = Matrix([[            cos(joint),            -sin(joint),           0,             length],
                     [ sin(joint)*cos(twist), cos(joint1)*cos(twist), -sin(twist), -sin(twist)*offset],
                     [ sin(joint)*sin(twist), cos(joint1)*sin(twist),  cos(twist),  cos(twist)*offset],
                     [                     0,                      0,           0,                  1]])
        return HTF
    
    def Rot_z(y):
        R_z = Matrix([[ cos(y), -sin(y), 0],
                      [ sin(y),  cos(y), 0],
                      [      0,       0, 1]])
        return(R_z)
    
    def Rot_y(p):
        R_y = Matrix([[  cos(p), 0, sin(p)],
                      [       0, 1,      0],
                      [ -sin(p), 0, cos(p)]])
        return(R_y)
    
    def Rot_x(r):
        R_x = Matrix([[ 1,      0,       0],
                      [ 0, cos(r), -sin(r)],
                      [ 0, sin(r),  cos(r)]])
        return(R_x)
    
    # Generation of homogeneous transformations
    HT0_1 = HTF_Matrix(joint1, twist0, length0, offset1).subs(dh)
    HT1_2 = HTF_Matrix(joint2, twist1, length1, offset2).subs(dh)
    HT2_3 = HTF_Matrix(joint3, twist2, length2, offset3).subs(dh)
    HT3_4 = HTF_Matrix(joint4, twist3, length3, offset4).subs(dh)
    HT4_5 = HTF_Matrix(joint5, twist4, length4, offset5).subs(dh)
    HT5_6 = HTF_Matrix(joint6, twist5, length5, offset6).subs(dh)
    HT6_G = HTF_Matrix(joint7, twist6, length6, offset7).subs(dh)
    
    # Matrix of the transformations
    HT0_G = HT0_1 * HT1_2 * HT2_3 * HT3_4 * HT4_5 * HT5_6 * HT6_G 
    
    # Position and orientation of the gripper at the end
    pos_x = req.poses[x].position.x
    pos_y = req.poses[x].position.y
    pos_z = req.poses[x].position.z
    
    # Obtaining the values of roll, pitch, yaw
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([req.poses[x].orientation.x, req.poses[x].orientation.y, req.poses[x].orientation.z, req.poses[x].orientation.w])
    
    # Between the DH and Gazebo parameters the rotation difference is compensated
    Rot_corr = Rot_z(pi) * Rot_y(-pi/2)
    Rot_rpy = Rot_x(r) * Rot_y(p) * Rot_z(y)
    Rot_rpy = Rot_rpy.evalf(subs={r: roll, p: pitch, y: yaw})
    R0_6 = Rot_rpy * Rot_corr
    
    # With the geometric IK method the joint angles are calculated
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
    
    theta5 = (atan2(sqrt(r13**2 + r33**2), r23)).evalf()
    theta4 = (atan2(r33, -r13)).evalf()
    theta6 = (atan2(-r22, r21)).evalf()
    ##
    ##
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]
    
    ## (OPTIONAL) YOUR CODE HERE!
    forw_kine = HT0_G.evalf(subs={joint1: theta1, joint2: theta2, joint3: theta3, joint4: theta4, joint5: theta5, joint6: theta6})
    
    ## End your code input for forward kinematics here!
    ########################################################################################
    
    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    #your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    your_wc = [wx, wy, wz]
    #your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    your_ee = [forw_kine[0, 3], forw_kine[1, 3], forw_kine[2, 3]]
    ########################################################################################
    
    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))
    
    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)
    
    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")
    
    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)


if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1
    
    test_code(test_cases[test_case_number])