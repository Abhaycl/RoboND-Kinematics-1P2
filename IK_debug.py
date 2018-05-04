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
    
    # Create symbols
    # Joint angle variables
    q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")
    # Link displacement variables
    d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
    # Link length variables
    a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")
    # Twist angle variables
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")
    # r: roll, p: pitch, y: yaw
    r, p, y = symbols("r p y")
    
    # DH parameterization for the KUKA KR210
    # KUKA KR210
    dh = {alpha0:     0, a0:      0, d1:  0.75, q1:      q1,
          alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
          alpha2:     0, a2:   1.25, d3:     0, q3:      q3,
          alpha3: -pi/2, a3: -0.054, d4:  1.50, q4:      q4,
          alpha4:  pi/2, a4:      0, d5:     0, q5:      q5,
          alpha5: -pi/2, a5:      0, d6:     0, q6:      q6,
          alpha6:     0, a6:      0, d7: 0.303, q7:       0}
    
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
    
    
    # Generation of homogeneous transformations
    HT0_1 = HTF_Matrix(alpha0, a0, d1, q1).subs(dh)
    HT1_2 = HTF_Matrix(alpha1, a1, d1, q2).subs(dh)
    HT2_3 = HTF_Matrix(alpha2, a2, d1, q3).subs(dh)
    HT3_4 = HTF_Matrix(alpha3, a3, d1, q4).subs(dh)
    HT4_5 = HTF_Matrix(alpha4, a4, d1, q5).subs(dh)
    HT5_6 = HTF_Matrix(alpha5, a5, d1, q6).subs(dh)
    HT6_G = HTF_Matrix(alpha6, a6, d1, q7).subs(dh)
    
    # Matrix of the transformations
    HT0_G = HT0_1 * HT1_2 * HT2_3 * HT3_4 * HT4_5 * HT5_6 * HT6_G 
    
    # Position and orientation of the gripper at the end
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
    
    # Obtaining the values of roll, pitch, yaw
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([req.poses[x].orientation.x, req.poses[x].orientation.y, req.poses[x].orientation.z, req.poses[x].orientation.w])
    
    # Between the DH and Gazebo parameters the rotation difference is compensated
    rot_corr = Rot_z.subs(y, pi) * Rot_y.subs(p, -pi/2)
    rot_rpy = rot_rpy * rot_corr
    rot_rpy = rot_rpy.subs({'r': roll, 'p': pitch, 'y': yaw})
    
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
    #s = wz - d_1
    
    # Use of the cosine law to calculate theta2 theta3 using A, B, C sides of the triangle
    # Side A
    A = sqrt(a_3**2 + d_4**2) # A = 1.50097
    # Side B
    B = sqrt((wz - d_1)**2 + r**2)
    # Side C
    C = a_2 # C = 1.25
    
    # Angle a (alpha)
    a = acos((C**2 + B**2 - A**2) / (2 * C * B))
    # Calculating theta 2
    theta2 = (pi/2) - a - atan2((wz - d_1), r)
    # Angle b (beta)
    b = acos((C**2 + A**2 - B**2) / (2 * C * A))
    # Calculating theta 3
    theta3 = (pi/2) - b - atan2(-a_3, d_4)
    
    # Calculating Euler angles from orientation
    R0_3 = HT0_1[0:3, 0:3] * HT1_2[0:3, 0:3] * HT2_3[0:3, 0:3]
    R0_3 = R0_3.evalf(subs={'q1': theta1, 'q2': theta2, 'q3': theta3})
    # Calculate inverse of R0_3
    R3_6 = R0_3.inv("LU") * rot_rpy
    
    # Calculating theta 4
    theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
    # Calculating theta 5
    theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
    # Calculating theta 6
    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
    
    ##
    ##
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]
    
    ## (OPTIONAL) YOUR CODE HERE!
    forw_kine = HT0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    
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