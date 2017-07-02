## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[theta1]: ./misc_images/theta1.png
[theta2]: ./misc_images/theta2.png
[theta3]: ./misc_images/theta3.png
[p1]: ./misc_images/1.png
[p2]: ./misc_images/2.png
[p3]: ./misc_images/3.png
[p5]: ./misc_images/5.png
[p7]: ./misc_images/7.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

With the explanation in the class video , we could find the DH parameters table is following:

joint | alpha | a | d | theta
--- | --- | --- | --- | ---
1 | 0       | 0     | 0.75  | q1
2 | -pi/2   | 0.35  | 0     | q2 - pi/2
3 |  0      | 1.25  | 0     | q3
4 | -pi/2   |-0.054 | 1.50  | q4
5 |  pi/2   | 0     | 0     | q5
6 | -pi/2   | 0     | 0     | q6
gripper | 0 | 0     | 0.303 | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

From the pose of gripper descibed by roll, pitch, yaw, we could get the rotation from urdf Frame to base inertial Frame :
``` python
R_x = Matrix([[1,         0,          0],
              [0, cos(roll), -sin(roll)],
              [0, sin(roll),  cos(roll)]])

R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
              [          0, 1,          0],
              [-sin(pitch), 0, cos(pitch)]])

R_z = Matrix([[cos(yaw), -sin(yaw),  0],
              [sin(yaw),  cos(yaw),  0],
              [       0,         0,  1]])

R0_G_URDF = R_x * R_y * R_z
```

We should eliminate the URDF format Frame setting by:
```
RC_z = Matrix([[cos(pi), -sin(pi), 0, 0],
               [sin(pi),  cos(pi), 0, 0],
               [      0,        0, 1, 0],
               [      0,        0, 0, 1]])

RC_y = Matrix([[ cos(-pi/2),  0, sin(-pi/2), 0],
               [          0,  1,          0, 0],
               [-sin(-pi/2),  0, cos(-pi/2), 0],
               [          0,  0,          0, 1]])

R_corr = RC_z * RC_y

R0_G = R0_G_URDF * R_corr.T
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

1. theta1 is caculated by the position wrist center by
    ``` python
    atan(y_wc, x_wc)
    ```
    as show in the picture, wrist center locates joint 4.
    ![theta1](theta1)

2. theta3 is caculate as show in the picture,
    ![theta3](theta3)
   because the a3 is negtive, so, in our case
   ```
   alpha - phi
   ```

3. theta4, theta6 can be caculated by
   ``` python 
    theta4 = atan2(-R4_6[1,2], -R4_6[0,2])
    theta6 = atan2(-R4_6[2,1], R4_6[2,0])
   ```

4. theta5 is the angle of link6 rotate from link4, so we could comput it by cross mulitply Z axes of link4 and link6
    ```python
    x0406 = N0_4.cross(N0_6)
    ```
    and then dot it with link5's Z axis, we can get it.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

This implementaion could pick up and drop objects correctly in every cases. here are some screenshots for demo:

![](p1)
![](p2)
![](p3)
![](p5)
![](p7)
