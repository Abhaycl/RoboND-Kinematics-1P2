# Kinematics Project Starter Code

The objective of this project is to program the robotic arm to pick up an element and place it in a defined place, using tools such as Gazebo and RViz.
<!--more-->

[//]: # (Image References)

[image0]: ./misc/forward_kinematics.png "Forward Kinematics"
[image1]: ./misc/kuka_sketch.png "Kuka KR210 Sketch"
[image2]: ./misc/calculate_moveit.png "Calculate Moveit"
[image3]: ./misc/urdf.png "Perspective Transform"
[image4]: ./misc/color_thresholding.jpg "Color Thresholding"
[image5]: ./misc/coordinate_transformations.jpg "Coordinate Transformations"
[image6]: ./misc/test_dataset_video.jpg "Test Dataset Video"
[image7]: ./misc/test_data_video.jpg "Test Data Video"
[image8]: ./misc/simulator.jpg "Simulator Parameters"
[image9]: ./misc/des1.jpg "Decision flow chart - Rocks"
[image10]: ./misc/des2.jpg "Decision flow chart - Forward"
[image11]: ./misc/des3.jpg "Decision flow chart - Stop"
[image12]: ./misc/des4.jpg "Decision flow chart - Return"
[image13]: ./misc/fidelity.jpg "Fidelity"

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




The Inverse Position Kinematics were addressed in this snippet from the [IK_debug.py]:

```python
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
```
