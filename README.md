# Kinematics Project Starter Code

The objective of this project is to program the robotic arm to pick up an element and place it in a defined place, using tools such as Gazebo and RViz.

---
<!--more-->

[//]: # (Image References)

[image0]: ./misc_images/forward_kinematics.png "Forward Kinematics"
[image1]: ./misc_images/kuka_sketch.png "Kuka KR210 Sketch"
[image2]: ./misc_images/calculate_moveit.png "Calculate Moveit"
[image3]: ./misc_images/urdf.png "URDF Coordinate System"
[image4]: ./misc_images/angles1.png "Calculate Angles1"
[image5]: ./misc_images/arctg.gif "Arc Tangente"
[image6]: ./misc_images/ik_equations.png "IK Equations"
[image7]: ./misc_images/arm_works1.png "Arm Works1"
[image8]: ./misc_images/arm_works2.png "Arm Works2"
[image9]: ./misc_images/error.png "Error"
[image10]: ./misc_images/kuka_kr210.png "Kuka Kr210"
[image11]: ./misc_images/paralelo.png "Parallel"
[image12]: ./misc_images/perpendicular.png "Perpendicular"
[image13]: ./misc_images/formula1.png ""
[image14]: ./misc_images/formula2.png ""
[image15]: ./misc_images/formula3.png ""
[image16]: ./misc_images/formula4.png ""
[image17]: ./misc_images/formula5.png ""
[image18]: ./misc_images/Rx.png "Rx"
[image19]: ./misc_images/Dx.png "Dx"
[image20]: ./misc_images/Rz.png "Rz"
[image21]: ./misc_images/Oi.png "Oi"
[image22]: ./misc_images/Dz.png "Dz"
[image23]: ./misc_images/Di.png "Di"
[image24]: ./misc_images/a.png "a"
[image25]: ./misc_images/O.png "O"
[image26]: ./misc_images/formula6.png ""
[image27]: ./misc_images/ik_equations1.png "IK Equations1"
[image28]: ./misc_images/ik_equations2.png "IK Equations2"
[image29]: ./misc_images/ik_equations3.png "IK Equations3"
[image30]: ./misc_images/ik_equations4.png "IK Equations4"

#### How build the project

```bash
1.  cd ~/catkin_ws
2.  catkin_make
```

#### How to run the project in demo mode

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

```bash
1.  cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
2.  rosrun kuka_arm IK_server.py
```

#### How to run the program with your own code

For the execution of your own code make sure the **demo** flag is set to _"false"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

```bash
1.  cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
2.  ./safe_spawner.sh
```

---

The summary of the files and folders int repo is provided in the table below:

| File/Folder                     | Definition                                                                                            |
| :------------------------------ | :---------------------------------------------------------------------------------------------------- |
| gazebo_grasp_plugin/*           | Folder that contains a collection of tools and plugins for Gazebo.                                    |
| kr210_claw_moveit/*             | Folder that contains all the configuration and launch files for using the kuka_arm with the MoveIt.   |
| kuka_arm/*                      | Folder that contains the kuka_arm package.                                                            |
| misc_images/*                   | Folder containing the images of the project.                                                          |
|                                 |                                                                                                       |
| IK_debug.py                     | File with the code to debug the project.                                                              |
| README.md                       | Contains the project documentation.                                                                   |
| README_udacity.md               | Is the udacity documentation that contains how to configure and install the environment.              |
| writeup_template.md             | Contains an example of how the practice readme documentation should be completed.                     |

---

### README_udacity.md

In the following link is the [udacity readme](https://github.com/Abhaycl/RoboND-Kinematics-1P2/blob/master/README_udacity.md), for this practice provides instructions on how to install and configure the environment.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

---
Some of the features of the Kuka Kr210 robotic arm that we are going to use are shown in the following image:

![alt text][image10]

![alt text][image0]
###### **Image**  **1** : Model represented by the foward_kinematics.launch file

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I get help from Lesson 2 and the project module to use the file forward_kinematics.launch to generate the kinematic sketch (image 2). The kr210.urdf.xacro file contains all the robot specific information like link lengths, joint offsets, actuators, etc. and it's necessary to derive DH parameters and create transform matrices.

![alt text][image1]
###### **Image**  **2** : Sketch to display links with offsets, lengths, and joint axes.

I get the twist angles:

|    |     |    |     |          |
| -- | --- | -- | --- | -------- |
| Z0 | ![alt text][image11] | Z1 | --> | a0 = 0   |
| Z1 | ![alt text][image12] | Z2 | --> | a1 = -90 |
| Z2 | ![alt text][image11] | Z3 | --> | a2 = 0   |
| Z3 | ![alt text][image12] | Z4 | --> | a3 = -90 |
| Z4 | ![alt text][image12] | Z5 | --> | a4 = 90  |
| Z5 | ![alt text][image12] | Z6 | --> | a5 = -90 |
| Z6 | ![alt text][image11] | ZG | --> | a6 = 0   |

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I get the following data from the file kr210.urdf.xacro

* J0 = (    0, 0,      0)
* J1 = (    0, 0,   0.33)
* J2 = ( 0.35, 0,   0.42)
* J3 = (    0, 0,   1.25)
* J4 = ( 0.96, 0, -0.054)
* J5 = ( 0.54, 0,      0)
* J6 = (0.193, 0,      0)
* JG = ( 0.11, 0,      0)

Where the following values are deducted:

* 0->1 ;  0.33 + 0.42 = 0.75
* 3->4 ;  0.96 + 0.54 = 1.5
* 6->G ; 0.193 + 0.11 = 0.303

Using the kr210.urdf.xacro file the below DH Parameter table was generated. Values were obtained by looking for the joints section in the xacro file; there using the sketch from image 2 distances from joint to joint were obtained and used as a(i-1) and d(i) values repective to their axis as provided in the Figure. Some values, like d(G) might need to obtained by the sum of multiple joint x, y, z values, in this case, the x value of joint 6 and the x value of the gripper joint.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
---- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 |   pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->G | 0 | 0 | 0.303 | 0

Given the DH data we will apply the following operations:

![alt text][image13]

In which ![alt text][image18] is a rotation matrix about the X axis by ![alt text][image19] is translation matrix along the X axis by ![alt text][image20] is a rotation matrix about the Z axis by ![alt text][image21], ![alt text][image22] is translation matrix along the Z axis by ![alt text][image23] and ![alt text][image24], a, ![alt text][image25] and d are D-H parameters of the robot. So we have:

![alt text][image14]

![alt text][image15]

![alt text][image16]

![alt text][image17]

The homogeneous transform for the DH coordinates system from joint i-1 to i is:

Let HT_i-1, i =

![alt text][image26]

Hence, the following matrix multiplication computes the final transformation matrix that gives the position and orientation of the robot’s end effector relative to its base.

HT0_G = HT0_1 * HT1_2 * HT2_3 * HT3_4 * HT4_5 * HT5_6 * HT6_G

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Inverse kinematics problem of a robot manipulator is finding the joint angles of the robot by having the position and orientation of the end effector of the robot. Inverse kinematics problem of a serial manipulator is more important than the forward kinematics, as it is essential to move the gripper of the robot to a required position with a defined orientation in order to, for instance, grab an object in that position and orientation.

Ok, now that I have the forward kinematics modeled it is time to tackle the real problem: calculate all joint angles for a trajectory, defined as an array of poses, calculated by MoveIt.

![alt text][image2]

###### Trajectory calculated by MoveIt.

The inverse kinematics problem was resolved analytically by dividing the problem in two parts: 1) Find the first 3 joint angles (counting from the base) from the pose position and 2) Find the remaining 3 wrist joint angles from the pose orientation.

##### Inverse Position Kinematics

First I have to find the position of the center of the wrist given the end-effector coordinate. But before that I need to account for a rotation discrepancy between DH parameters and Gazebo (URDF).

![alt text][image3]

For that I've created a correctional rotation composed by a rotation on the Z axis of 180° (π) followed by a rotation on the Y axis of -90 (-π/2).

```python
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            rot_corr = Rot_z.subs(y, pi) * Rot_y.subs(p, -pi/2)
```

Finally I've performed a translation on the opposite direction of the gripper link (that lays on the Z axis) to find the wrist center.

Calculate the wrist center by applying a translation on the opposite direction of the gripper, from the DH parameter table we can find that the griper link offset (d7) is 0.303m.

```python
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
```

Once the wrist center (WC) is known we can calculate the first joint angle with a simple arctangent.

![alt text][image4]

![alt text][image5]

Where: s = wz - d_1

With the help of the Law of Cosines I've calculated the values for angles alpha and bet.

![alt text][image6]

![alt text][image27]

![alt text][image28]

![alt text][image29]

![alt text][image30]

```python
            # Calculating theta 1
            theta1 = atan2(wy, wx)
            
            # For the evaluation of the angles we apply the law of cosine
            # Calculating radius
            r = sqrt(wx**2 + wy**2) - a_1
            
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
			theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

In order to obtain the transformation and rotation matrices, I decided to utilize functions to generate all of the different matrices. This is shown in the [IK_server.py] snippet below.

```python
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
```

These allowed the code to easily create the many transformation and rotation matrices by calling the functions, while still being outside of the handle_calculate_IK function. Another advantage was to generate all the transformation and rotation matrices outside the forloop to prevent them being generated constantly which would decrease performance and effectiveness. Further, I tried to leverage the DH parameters as much as possible given that they were already created and stored.

Possibly, due to computer performance, it was rather slow still and while I tried implementing a class structure to the code.

![alt text][image7]
![alt text][image8]

### Observations, possible improvements, things used

While debugging the code many times (long times due to slow performance), I noticed that the code does not respond well when the planned path is relatively abnormal and navitates far away to grab a can or to move towards the bucket. Not sure why this happens but when normal trajectories are given the code performs well. Not sure if it'll require calibration or more statements to make it smarter and discern the correct path to take on that kind of situation.