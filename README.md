# Kinematics Project Starter Code

The objective of this project is to program the robotic arm to pick up an element and place it in a defined place, using tools such as Gazebo and RViz.
<!--more-->

[//]: # (Image References)

[image0]: ./misc/rover_image.jpg "Rover"
[image1]: ./misc/data_test.jpg "Data Test"
[image2]: ./misc/pictures_for_calibration.jpg "Pictures For Calibration"
[image3]: ./misc/perspective_transform.jpg "Perspective Transform"
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

I get help from Lesson 2 and the project module to use the file forward_kinematics.launch to generate the kinematic sketch.

![alt text][image0]