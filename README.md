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

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I get help from Lesson 2 and the project module to use the file forward_kinematics.launch to generate the kinematic sketch (image 2). The kr210.urdf.xacro file contains all the robot specific information like link lengths, joint offsets, actuators, etc. and it's necessary to derive DH parameters and create transform matrices.

![alt text][image0]

###### **Image**  **1** : Model represented by the foward_kinematics.launch file

![alt text][image1]
###### **Image**  **2** : Sketch to display links with offsets, lengths, and joint axes.

Joint 1
* a0 = 0, since this is the base link.
* d1​ = link2_[z] = 0.75

Joint 2
* a1 = link_2[x] = 0.35
* d2​ = 0, since X1 and X2 are perpendicular.

Joint 3
* a2 = link_3[z] = 1.25
* d3​ = 0, since X2 and X3 are coincident.

Joint 4
* a3 = link_3[z] - link_5[z] = 2 – 1.9464 = 0.0536
* d4​ = link_5[x] - link_3[x] = 1.8499 – 0.3485 = 1.5014

Joint 5
* a4 = 0, since O4 and O5 are coincident.
* d5​ = 0, since X4 and X5 are coincident.

Joint 6
* a5 = 0, since O5 and O6 are coincident.
* d6​ = 0, since X5 and X6 are coincident.

Joint 7 (Gripper Joint)
* a6 = 0, since Z6 is coincident with Z7.
* d7​ = link_gripper[x] - link_5[x] = 2.1529 – 1.8499 = 0.303

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Using the kr210.urdf.xacro file the below DH Parameter table was generated. Values were obtained by looking for the joints section in the xacro file; there using the sketch from image 2 distances from joint to joint were obtained and used as a(i-1) and d(i) values repective to their axis as provided in the Figure. Some values, like d(EE) might need to obtained by the sum of multiple joint x, y, z values, in this case, the x value of joint 6 and the x value of the gripper joint.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | 0.0536 | 1.5014 | q4
4->5 |   pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

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
            # Calculating the Rotation Matrix for the Gripper
            rot_rpy = rot_rpy * rot_corr
            rot_rpy = rot_rpy.subs({'r': roll, 'p': pitch, 'y': yaw})
            #
            #
            # Calculate joint angles using Geometric IK method
            d_7 = dh[d7]
            wx = px - (d_7 * rot_rpy[0,2])
            wy = py - (d_7 * rot_rpy[1,2])
            wz = pz - (d_7 * rot_rpy[2,2])
```

Once the wrist center (WC) is known we can calculate the first joint angle with a simple arctangent.

![alt text][image4]
![alt text][image5]

With the help of the Law of Cosines I've calculated the values for angles alpha and bet.

![alt text][image6]

```python
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