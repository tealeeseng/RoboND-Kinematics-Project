## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace. [done]
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  [done]
3. Experiment with the forward_kinematics environment and get familiar with the robot. [done]
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193). [done]
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view). [done]
6. Fill in the `IK_server.py` with your Inverse Kinematics code. [done]


[//]: # (Image References)

[image1]: ./misc_images/Robo-DH.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/IK-slow-3rdcan.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Following is the schematic to derive DH Parameters. Right hands rule is useful to layout directions of X and Z axis in each joint.
![alt text][image1]

With schematic above and kr210.urdf.xacro, we can derive DH table as following.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.33+0.42 | 0+q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | 0+q3
3->4 | - pi/2 | -0.054 | 1.5 | 0+q4
4->5 |   pi/2 | 0 | 0 | 0+q5
5->6 | - pi/2 | 0 | 0 | 0+q6
6->EE | 0 | 0 | 0.193+0.11 | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


```python
    #DH table
    s = {alpha0:    0,   a0:    0,  d1:  0.75,  q1: q1,
         alpha1:-pi/2.,  a1: 0.35,  d2:     0,  q2: q2-pi/2.,
         alpha2:    0,   a2: 1.25,  d3:     0,  q3: q3,
         alpha3:-pi/2.,  a3:-0.054, d4:  1.5,  q4: q4,
         alpha4: pi/2.,  a4:     0, d5:     0,  q5: q5,
         alpha5:-pi/2.,  a5:     0, d6:     0,  q6: q6,
         alpha6:    0,   a6:     0, d7: 0.303,  q7: 0}

    #### Homogeneous Transforms
    def TF_Matrix(alpha, a, d, q):
        TF = Matrix(
            [[            cos(q),           -sin(q),          0,                a],
             [ sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
             [ sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
             [                 0,                 0,            0,              1]])
        return TF

    ###
    # Transformation matrix at each joint.    
    T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
    T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
    T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
    T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
    T6_EE =TF_Matrix(alpha6, a6, d7, q7).subs(s)

    # end to end transformation.
    T0_EE = T0_1 * T1_2 * T2_3* T3_4* T4_5*T5_6* T6_EE

```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
As this nverse Kinematic problem can be divided into Inverse Position kinematics and inverse Orientation Kinematics, 
all we need is to find out wrist centre as below
```python
    r, p, y = symbols('r p y')

    ROT_x = Matrix([[1,     0,      0],
                    [0, cos(r), -sin(r)],
                    [0, sin(r), cos(r)]])

    ROT_y = Matrix([[cos(p),    0,  sin(p)],
                    [   0,      1,      0],
                    [-sin(p),   0,  cos(p)]])

    ROT_z = Matrix([[cos(y), -sin(y), 0],
                    [sin(y), cos(y),  0],
                    [   0,      0,    1]])

    ROT_EE = ROT_z * ROT_y * ROT_x

    Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

    ROT_EE = ROT_EE * Rot_Error
    ROT_EE = ROT_EE.subs({'r':roll, 'p':pitch, 'y': yaw})

    EE = Matrix([[pos.x],
                 [pos.y],
                 [pos.z]])

    WC = EE - (0.303) * ROT_EE[:, 2]
```

First 3 thetas can be derived as following with respective to wrist centre.
```python

    # Calculte joint angles using Geometric IK method
    # More information can be found in the Inverse Kinematics with Kuka KR210
    theta1 = atan2(WC[1], WC[0])

    # SSS triangle for theta2 and theta3
    side_a = 1.501
    side_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] -0.75),2))
    side_c = 1.25

    #  cosine law
    angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / (2*side_b*side_c))
    angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / (2*side_a*side_c))
    angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c) / (2*side_a*side_b))

    theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1]) - 0.35)
    theta3 = pi/2 - (angle_b + 0.036) #0.036 accounts for sag in link4 of -0.054m


```

Whereas the last 3 thetas can be derived as following
```python

    R0_3 = T0_1[0:3, 0:3]* T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})

    R3_6 = R0_3.inv("LU") * ROT_EE

    #Euler angles from rotation matrix
    #More information can e found in the Euler Angles from a Rotation Matrix section
    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2( -R3_6[1,1], R3_6[1,0])

```


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

My IK_server.py can be found at https://github.com/tealeeseng/RoboND-Kinematics-Project/blob/master/kuka_arm/scripts/IK_server.py
I got most of the sample code from https://www.youtube.com/watch?v=Gt8DRm-REt4 and trial runs in https://github.com/tealeeseng/RoboND-Kinematics-Project/blob/master/IK_debug.py.
until testing result matched accordingly.

In IK_server.py, I moved code with sympy before the for loop. It does reasonable well for certain IK but also spent 7-10+ seconds on other IK path.
![alt text][image3]

With reinforcement learning, Deep Q network and sensors, I wonder whether we can train IK functions like Baxter robots, https://www.youtube.com/watch?v=nA-J0510Pxs


