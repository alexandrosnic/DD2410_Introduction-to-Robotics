#### Introduction
The final project consists of a simulation in [Gazebo](http://gazebosim.org/) of a [TIAGo robot](http://tiago.pal-robotics.com/) in an apartment. The robot is equipped with several onboard sensors and a manipulator which enable it to navigate and interact with the environment through simple manipulation tasks. The goal of this project is to implement a mission planner for TIAGo to execute a set of tasks.

#### Format
Unlike in previous assignments, in this project you are not asked to develop a software package. Instead, this is a system integration project, in which all necessary modules are provided (planning, sensing, navigation...) and you have to integrate them so that the robot can carry out a mission. Unfortunately, just like in many real system integration projects, most of the packages you are given come with very little documentation and the developers are not around any more...

You are expected to implement and show a working solution consisting of two parts:

1. A mission planner, within the corresponding file:
a. "sm_students.py" in which you have to implement a high-level state machine (SM) triggering the right sequence of actions for the robot to achieve the final goal. In order to come up with this sequence, look at the videos provided and go through all the topics, services and actions offered by the running system. If you need the robot to do a backflip, most likely there's a service called /backflip_jump or similar. An example of state machine is provided within the working package.
b. "bt_students.py" contains a behavior tree (BT) equivalent to the state machine above.
2. A launch file "launch_project.launch" where you have to deploy and connect the nodes/components of the system to carry out the tasks. For each different task you will have to uncomment the  nodes and provide them with the parameters required (substitute place_holderX with meaninful names).

Use the same node in the launch file to launch either the SM or BT just by changing the name of the python script being called, depending on the task your solving

```
<node pkg="robotics_project" type="sm_students.py" name="logic_state_machine" output="screen">
```

The files mentioned are under the robotics_project package, available for you in the repository you are about to clone. You do not have to modify any other file than these three. 

There are three different tasks with increasing difficulty levels, which will give you a corresponding grade. The reasonable way to go is to start with E and move on from there, since the higher levels build conceptually on top of the previous ones (with some changes in the implementation).

Along with the demos, you will be questioned about basic concepts of the solutions you have implemented (see examples below). The tasks are considered solved only if you are able to answer these questions during the presentation.

#### Task E: Pick&Carry&Place without sensory input
In this scenario, TIAGo is expected to pick an Aruco cube from a known pose on top of table 1, navigate towards table 2 behind it and place the object there. The cameras and laser scan cannot be used for this level.

[E](https://j.gifs.com/k8PMpK.gif)

Implement a state machine which goes through the following main states:

1. Complete picking task
2. Carry cube to second table
3. Complete placing task
Obs: use the ROS tools (rostopic, rosmsg, rosservice, rosrun tf view_frames, etc) to explore the system and figure out which module does what. You will realize this is the most time-consuming part of this level. Once you are familiar with the project, implementing the solution will be straight forward.

Evaluation:

1. Show a working simulation and be able to explain your implementation
2. Be able to reason about this solution, i.e: Can the mission succeed if the cube is displaced before being picked? What if table 2 is moved?

#### Task C: Pick&Carry&Place with visual sensing
The task is the same as above. However this time the camera sensor in TIAGo's head has to be used to detect the cube. After, compute a grasp, transport the marker and verify that it has been placed on the second table. You will implement this logic in the form of a behavior tree this time.

(https://eu.nv.instructuremedia.com/fetch/QkFoYkIxc0hhUU4xd0Fvd2JDc0gxV2w2WGc9PS0tOWUzYmU3YmYzNDBiMjU5OTM0YzIxN2RmYjJhZmQwNDM2MGIyZDRiZg.mp4)

Implement a behavior tree which goes through the following main states:

1. Detect cube
2. Complete picking task 
3. Carry cube to second table
4. Complete placing task
5. Cube placed on table?
a. Yes: end of task
b. No: go back to initial state in front of table 1
Evaluation:

1. Show a working simulation and be able to explain your implementation
2. Be able to reason about this solution, i.e: Can the mission succeed if the cube is displaced before being picked? What if table 2 is moved? Would the robot be able to transport several cubes (one at a time) with the given solution?  

#### Task A: Pick&Carry&Place with sensing and navigation
Pick&Carry&Place with visual sensing and navigation: in this third level, the robot starts in an unknown pose and must make use of its sensors and a prior map of the room to transport the cube safely among rooms.

<iframe src='//gifs.com/embed/robotics-assignment5-e-k8PMpK' frameborder='0' scrolling='no' width='640px' height='360px' style='-webkit-backface-visibility: hidden;-webkit-transform: scale(1);' ></iframe>

Implement a behavior tree that goes through the following main states:

1. Robot has localized itself in the apartment
2. Navigation to picking pose
3. Cube detected
4. Complete picking task 
5. Navigation with cube to second table
6. Complete placing task
7. Cube placed on table?
a. Yes: end of mission
b. No: go back to state 2 and repeat until success. For this, you need to respawn the cube to its original pose in case it has fallen.
Obs 1: At any time during the navigation, a bad-intentioned TA might kidnap your robot again. Your behavior tree must be able to detect this and react to it so that the robot always knows its true position. Kidnap the robot yourself during your development to test your solution (the robot can be moved in Gazebo manually).

Obs 2: The robot uses a particle filter for localization. Use the distribution of the particles to know when the filter has converged. Other solutions will not be accepted.

Evaluation:

1. Show a working simulation and be able to explain your implementation
2. Be able to reason about this solution, i.e: Why do we ask you to make the robot spin to help the AMCL? What does the distribution of particles tell us? Why does AMCL fail to converge sometimes? When to use/avoid timers while waiting for a result from an action server.

#### Install
The following instructions are for the PCs in the lab rooms, which have already been set up for you.

```
# Download the repository:
cd ~/catkin_ws/src/
git clone https://github.com/ignaciotb/robi_final_project.git

# Build the project:
cd ~/catkin_ws
catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=RelWithDebInfo
source devel/setup.bash
```

To run the project in personal computers, your own missing system dependencies have to be met. A hint on how to solve this can be found in this [README] (https://github.com/ignaciotb/robi_final_project/blob/master/README.md) but the setups will vary for each of your installations and we will not offer support for this (Google is your friend here). As an extra advice, stay away from virtual machines and make sure your laptop can handle this workload

#### Launch the simulation
To run the project, execute the following commands in two terminals:

```
# Launch Gazebo and RViZ
roslaunch robotics_project gazebo_project.launch

# Deploy the system and start the simulation
roslaunch robotics_project launch_project.launch
```

Wait for Gazebo and RViZ to show up before running the second command.

While developing and testing, you can stop (Ctrl+C) the launch_project.launch script and relaunch it without having to relaunch the gazebo part. If you do not want to run the Gazebo graphical interface (client) in order to speed things up, you can set the "gzclient" variable in the launch file to false.

If everything works out, you should see the robot moving around the apartment, folding its arm, approaching a chair and lowering its head. This dummy example (also available as a BT) shows you how to call services and actions and interact with topics from your mission planner. They are good skeletons to start to develop your own solutions.

#### Dealing with probabilities
You might experience that the simulation fails some times despite the fact that all components are correctly implemented (much like in a real system). MoveIt! may fail to compute a trajectory for the robot arm while picking/placing (i.e Pick result: PLANNING_FAILED) or the Navigation stack might get the robot stuck in a loop while trying to reach a waypoint.

Handling these errors is not requested beyond logging them and showing them on terminal from your mission planner. If they occur during the demo, you will be given another try to run the full simulation.