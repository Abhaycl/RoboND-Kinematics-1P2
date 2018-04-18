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

![alt text][image0]

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
Run the simulator Roversim_x86_64.exe

---

The summary of the files and folders int repo is provided in the table below:

| File/Folder                     | Definition                                                                                            |
| :------------------------------ | :---------------------------------------------------------------------------------------------------- |
| gazebo_grasp_plugin/*           | Folder that contains a collection of tools and plugins for Gazebo.                                    |
| kr210_claw_moveit/*             | Folder that contains all the configuration and launch files for using the kuka_arm with the MoveIt.   |
| kuka_arm/*                      | Folder that contains the kuka_arm package.                                                            |
| misc_images/*                   | Folder containing the images of the project.                                                          |
|                                 |                                                                                                       |
| IK_debug.py                     |                                                                                                       |
| README.md                       | Contains the project documentation.                                                                   |
| README_udacity.md               | Is the udacity documentation that contains how to configure and install the environment.              |
| writeup_template.md             | Contains an example of how the practice readme documentation should be completed.                     |

---

### README_udacity.md

In the following link is the [udacity readme] https://github.com/Abhaycl/RoboND-Kinematics-1P2/blob/master/README_udacity.md , for this practice provides instructions on how to install and configure the environment.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/forward_kinematics.JPG
[image2]: ./misc_images/kuka_KR210_sketch.jpg
[image3]: ./misc_images/kinematics_analysis_WC.jpg
[image4]: ./misc_images/ik_equations_v1_3.JPG
[image5]: ./misc_images/works.JPG
[image6]: ./misc_images/works3.JPG
[image7]: ./misc_images/error.JPG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I get help from Lesson 2 and the project module to use the file forward_kinematics.launch to generate the kinematic sketch.

![alt text][image1]





### The goals / steps of this project are the following:

- Training / Calibration:
  - Download the simulator and take data in "Training Mode"
  - Test out the functions in the Jupyter Notebook provided
  - Add functions to detect obstacles and samples of interest (golden rocks)
  - Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map. The `output_image` you create in this step should demonstrate that your mapping pipeline works.
  - Use `moviepy` to process the images in your saved dataset with the `process_image()` function. Include the video you produce as part of your submission.

- Autonomous Navigation / Mapping
  - Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook).
  - Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.
  - Iterate on your perception and decision function until your rover does a reasonable job of navigating and mapping. (Mapping 40% of the map, at 60% Fidelity, and locate at least 1 rock sample).

#### We execute the functions that have been provided in the notebook, first with the test data provided, then with the data that I have recorded from the simulator. Add/modify functions to allow color selection of obstacles and rock samples.

- Collection of test data

![alt text][image1]

- Images for the calibration

![alt text][image2]

- Perspective Transform

![alt text][image3]

- Color Thresholding

![alt text][image4]

- Coordinate Transformations

![alt text][image5]

#### I complete the process_image() function with the appropriate analysis steps for mapping pixels by identifying navigable terrain, obstacles and rock samples on a world map. I run process_image() with the test data using the provided movie functions to create a video output as a result.

I have made some modifications to different functions. All details are shown in the jupyter notebook 'Rover_Project_Test_Notebook.ipynb'.

- Output Image results with test_dataset provided

![alt text][image6]

- Output Image results with my recorded test data

![alt text][image7]

### Autonomous Navigation and Mapping

#### Fill in the perception_step() (at the bottom of the perception.py script) and decision_step() (in decision.py) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

#### - A. perception.py modifications:

1. These color_thresh function modifications make it so that it outputs all 3 thresholds, one for navigable path, rock samples, and rock samples; respectively. Thought, red and green thresholds higher than 100 and blue threshold lower than 50 do to recognize the yellow pixels from the rock samples.

We define several thresholds to identify the different objects, such as the path, obstacles and rocks to be collected, these thresholds are composed of a maximum and minimum range.
```python
def color_thresh(img, min_path=np.array([159, 159, 159]), max_path=np.array([255, 255, 255]),
                      min_rock=np.array([93, 173, 131]), max_rock=np.array([98, 255, 182]),
                      min_obst=np.array([0, 0, 0]), max_obst=np.array([140, 140, 140])):
```

Create an array of zeros same xy size as img, but single channel.
```python
    color_select_path = np.zeros_like(img[:,:,0])
    color_select_rock = np.zeros_like(img[:,:,0])
    color_select_obst = np.zeros_like(img[:,:,0])
```

Require that each pixel be above all three threshold values in RGB above_thresh will now contain a boolean array with "True" where threshold was met. I changed the RGB to HSV, so we could better detect the rocks. and take into account the maximum and minimum values of the different thresholds.
```python
    # Threshold for navigable path
    above_thresh = ((img[:, :, 0] > min_path[0]) & (img[:, :, 0] <= max_path[0])) \
                 & ((img[:, :, 1] > min_path[1]) & (img[:, :, 1] <= max_path[1])) \
                 & ((img[:, :, 2] > min_path[2]) & (img[:, :, 2] <= max_path[2]))
    # Threshold for rocks
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only yellow colors
    between_thresh = ((hsv[:, :, 0] > min_rock[0]) & (hsv[:, :, 0] <= max_rock[0])) \
                   & ((hsv[:, :, 1] > min_rock[1]) & (hsv[:, :, 1] <= max_rock[1])) \
                   & ((hsv[:, :, 2] > min_rock[2]) & (hsv[:, :, 2] <= max_rock[2]))
    # Threshold for obstacles
    below_thresh = ((img[:, :, 0] > min_obst[0]) & (img[:, :, 0] <= max_obst[0])) \
                 & ((img[:, :, 1] > min_obst[1]) & (img[:, :, 1] <= max_obst[1])) \
                 & ((img[:, :, 2] > min_obst[2]) & (img[:, :, 2] <= max_obst[2]))
```

Index the array of zeros with the boolean array and set to 1.
```python
    color_select_path[above_thresh] = 1
    color_select_rock[between_thresh] = 1
    color_select_obst[below_thresh] = 1
```

Return the binary images
```python
    return color_select_path, color_select_rock, color_select_obst
```

2. Changes to the perspect_transform function generate a second output to consider the outside field of view, which would otherwise be considered part of the obstacles without this method. Also, a mask is obtained to further narrow the field of view

```python
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    # Warped image
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    # Mask
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))

    # Return warped
    return warped, mask
```

3. Changes to the perception_step() or process_image() function have been made to apply all the functions previously stated to provide a complete computer vision image that can be used to tell the rover where it can travel and where it can't, as well as adding the ability to detect rock samples. I also applied the to_polar_coords() function to the rock 'x' and 'y' pixels to provide the rover distance and direction to where the rock samples are, for steering guidance.

Perform perception steps to update Rover
```python
def perception_step(Rover):
    # Get the images from the rover's camera
    image = Rover.img
```
	
Define source and destination points for perspective transform, define calibration box in source (actual) and destination (desired) coordinates these source and destination points are defined to warp the image to a grid where each 10x10 pixel square represents 1 square meter.

A bottom offset is set to account for the fact that the bottom of the image is not the position of the rover but a bit in front of it this is just a rough guess.
```python
    # The destination box will be 2*distance_size on each side
    distance_size = 5
    bottom_offset = 6
    scale = 2 * distance_size
    source = np.float32([[14, 140], [301 ,140], [200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - distance_size, image.shape[0] - bottom_offset],
                              [image.shape[1]/2 + distance_size, image.shape[0] - bottom_offset],
                              [image.shape[1]/2 + distance_size, image.shape[0] - scale - bottom_offset], 
                              [image.shape[1]/2 - distance_size, image.shape[0] - scale - bottom_offset],])
```

Apply perspective transform to obtain the warped image and mask.
```python
    warped, mask = perspect_transform(Rover.img, source, destination)
```

Apply color threshold to identify navigable terrain/obstacles/rock samples.
```python
    threshed_path, threshed_rock, threshed_obst = color_thresh(warped)
    obst_map = np.absolute(np.float32(threshed_obst)) * mask
```

Update Rover vision image (this will be displayed on left side of screen).
```python
    Rover.vision_image[:,:,0] = obst_map * 255
    Rover.vision_image[:,:,1] = threshed_rock * 255
    Rover.vision_image[:,:,2] = threshed_path * 255
```

Convert map image pixel values to rover-centric coords.
```python
    obst_xpix, obst_ypix = rover_coords(obst_map)
    rock_xpix, rock_ypix = rover_coords(threshed_rock)
    path_xpix, path_ypix = rover_coords(threshed_path)
```

Convert rover-centric pixel values to world coordinates.
```python
    world_size = Rover.worldmap.shape[0]
    obst_xworld, obst_yworld = pix_to_world(obst_xpix, obst_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, Rover.scale)
    rock_xworld, rock_yworld = pix_to_world(rock_xpix, rock_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, Rover.scale)
    path_xworld, path_yworld = pix_to_world(path_xpix, path_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, Rover.scale)
```

Update Rover worldmap (to be displayed on right side of screen).
```python
    # Don't corrupt perspective and thus map fidelity.
    if (Rover.roll < 1.0 or Rover.roll < 359.0) and (Rover.pitch < 1.0 or Rover.pitch < 359.0):
        Rover.worldmap[obst_yworld, obst_xworld, 0] = 255
        Rover.worldmap[rock_yworld, rock_xworld, 1] = 255
        Rover.worldmap[path_yworld, path_xworld, 2] = 255
```

Convert rover-centric pixel positions to polar coordinates.
```python
    obst_dist, obst_angles = to_polar_coords(obst_xpix, obst_ypix)
    rock_dist, rock_angles = to_polar_coords(rock_xpix, rock_ypix)
    path_dist, path_angles = to_polar_coords(path_xpix, path_ypix)
    rover_dist, rover_angles = to_polar_coords(Rover.pos[0], Rover.pos[1])
```

Update Rover pixel distances and angles.
```python
    Rover.obst_dists = obst_dist
    Rover.obst_angles = obst_angles
    Rover.rock_dists = rock_dist
    Rover.rock_angles = rock_angles
    Rover.nav_dists = path_dist
    Rover.nav_angles = path_angles
    Rover.rover_dists = rover_dist
    Rover.rover_angles = rover_angles
```

The initial position of the rover is defined when it starts to move.
```python
    # Keep the initial position
    if Rover.pos_ini:
        Rover.pos_ini = False
        Rover.home_dists = rover_dist
        Rover.home_angles = rover_angles
    
    # The values of the rover are returned
    return Rover
```

#### - B. decision.py modifications:

1. Made these changes to the decision_step() function to provide the extra capability to locate and steer towards rock samples when found, stop when near a sample, and pickup sample when it has stopped in front of the rock sample.

First we check if there is navigable terrain.

- Location of rocks:

First we check if there is navigable terrain:

The decisions to make are: when the rover finds a rock, it looks at the angle from where the rock is, to know if it has to accelerate more or accelerate less and when the rover is at a safe distance it stops to collect the sample.

![alt text][image9]
```python
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # The rover has located a sample of rock
        if len(Rover.rock_angles) != 0:
```

- The rover moves forward:

If the rover is moving it will check if there is a navigable road where it will check if it has a minimum speed to be able to increase the acceleration or if it has to maintain the speed, if there is no navigable road it will start stopping.

![alt text][image10]
```python
        # If there is a suitable navigable road, check for Rover.mode status
        elif Rover.mode == 'forward' and len(Rover.rock_angles) == 0 and Rover.near_sample == 0:
```

- The rover stops:

If the rover starts stopping, check the speed, if its speed is higher than a minimum then it will brake otherwise if there is no navigable road it will turn and if not it will advance or if it has collected all the rocks it will return.

![alt text][image11]
```python
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop' and len(Rover.rock_angles) == 0:
```

- The rover returns to the starting point:

Once all the rocks are collected to return the rover checks if it is near the origin to stop otherwise check that the distance in which it is less than the position of the last rock collected in which it checks if there is a navigable road to continue advancing, if there was no navigable road the rover would turn. If the rover were in the same position as the last rock collected, it would identify which area it was in to make the correct turn and move forward to return along the proper path.

![alt text][image12]
```python
        # If we have collected all the samples, return home
        elif Rover.mode == 'return':
```

When the rover gets stuck and a certain amount of time has elapsed the stuck() function will be activated which will cause the rover to reverse while rotating, also rarely in large areas the rover makes circles so we will correct the trajectory a little.
```python
# The rover can get stuck with some object, it will decelerate while it turns
def stuck(Rover, mode, throttle):
    # In large areas the rover seldom makes circles, with this we deflect it
    if (Rover.vel >= Rover.max_vel and Rover.steer == 15) or (Rover.vel >= Rover.max_vel and Rover.steer == -15):
        Rover.steer = 0
    
    # If the rover gets stuck with an object
    if(Rover.vel > 0.05 and Rover.mode != mode):
        Rover.stuck_time = 0
    elif (Rover.total_time > 1 and Rover.vel == 0.0):
        if Rover.stuck_time == 0:
            Rover.stuck_time = Rover.total_time
        elif (Rover.total_time - Rover.stuck_time) > 1:
            # The rover's going backwards
            Rover.throttle = throttle
            # Wait for 350 milliseconds
            time.sleep(.350)
            # If there are valid values from the navigation path
            if (np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)) >= 0 or (np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) < 0):
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            else:
                Rover.steer = 15
```

#### - C. drive_rover.py modifications:

1. Made these changes to the init() function to provide the extra variables to the rover for storing rock sample distance and angles, along with a string variable that's used to prompt in different situations for testing and debugging purposes.

I change the acceleration, stop and start thresholds of the rover.
```python
class RoverState():
    def __init__(self):
        self.throttle_set = 0.6 # Throttle setting when accelerating
        self.stop_forward = 200 # Threshold to initiate stopping
        self.go_forward = 1000 # Threshold to go forward again
```

I add more parameters to the rover that contain the distances and angles of the rocks to be collected, obstacles, the initial position and the rover, rover stuck time.
```python
        self.pos_ini = True # Set initial values for the position
        self.pos_fin = True # Set final values for the position
        self.start_pos = None # Start position (x, y)
        self.dist_fin = None # Final position
        self.stuck_time = None # Stuck time
        self.rock_dists = 0 # Rock distances of navigable terrain pixels
        self.rock_angles = 0 # Rock angles of navigable terrain pixels
        self.obst_dists = None # Obstacle distances of navigable terrain pixels
        self.obst_angles = None # Angles of navigable terrain pixels
        self.home_dists = None # Distance from home position
        self.home_angles = None # Angles from home position
        self.rover_dists = None # Angles of navigable terrain pixels
        self.rover_angles = None # Angles from home position
        self.scale = 30 # To adjust the rover's vision sizes
```

In the telemetry function def telemetry(sid, data): we add the rover parameters with the initial position.
```python
        if Rover.start_pos is None:
            Rover.start_pos = Rover.pos
            Rover.stuck_time = Rover.total_time
```

Visualize complementary information in the simulator - speed, mode and FPS
```python
            if Rover.picking_up:
                mode = 'picking up'
            else:
                mode = Rover.mode
            cv2.putText(Rover.vision_image, "Mode: {}".format(mode), (2, Rover.vision_image.shape[0] - 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(Rover.vision_image, "Current FPS: {}".format(fps), (2, Rover.vision_image.shape[0] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
```

#### - D. supporting_functions.py modifications:

In the function def update_rover(Rover, data): we add the following lines of code to display more information about the position of the rocks to be collected.

```python
    'samples collected:', Rover.samples_collected, 'rock angles', Rover.rock_angles,
    'rock distance', Rover.rock_dists, 'Samples positions', Rover.samples_pos)
```

In the create_output_images(Rover) function: we add the total number of rocks to collect
```python
    cv2.putText(map_add,"Rocks: "+str(Rover.samples_to_find), (0, 55), 
                cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
```

#### - E. Fidelity:

As we can see from the image below, the rover maps at least 40% of the environment with a percentage greater than 60% fidelity (accuracy) against the ground truth.

![alt text][image13]

#### Launching in autonomous mode your rover can navigate and map autonomously. Explain your results and how you might improve them in your writeup.

In autonomous mode, I managed to map at least 40% with at least 60% fidelity. Also, some capability to detect, navigate towards rock samples, and pick them up was added. There are instances where it would crash against the rocks in the middle of the map, sometimes getting stuck. Also, sometimes it stays in a loop going to the same places over and over, or just stays around in circles looking for a greater navigable path.

Would further optimization be pursued, it would be in the better obstacle detection area (specially rocks in the middle of the map, not big enough to be perceived as obstacle), along with better decision making after it notices it stayed in a loop, and running in circles.

#### Running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines! Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by drive_rover.py) in your writeup when you submit the project so your reviewer can reproduce your results.

![alt text][image8]

### Observations, possible improvements, things used

I applied the perception step functions to provide computer vision leveraging the Rover data set up in the drive_rover.py; from color_thresh() to pix_to_world(). Modified slightly the color_thresh() function to output the 3 threshold required to detect obstacles, path, and rock samples. Furthermore, added directional data to Rover for rock samples by applying to_polar_coords() to rock pixels.

Also, in the decision_step() function, I added simple if and elif chain events to trigger stopping and slower speeds upon detecting rock samples, with some steering towards the rock pixel angles provided by the perception step, the movements are a little abrupt. Occasionally, it stops on top of the rock samples and gets stuck. The rover sometimes, stays running around in circles, not sure if this is due to the frames per seconds (FPS) or if it's perception_step() that could be improved.

I would improve time optimization to have the rover run and stop more efficiently. Furthermore, make the rover go back to where it started; this might be tight together with position awareness that would also prevent the rover from running around in circles.