# Introduction to Robotics (DD2410)

## Assignment 1: Introduction to ROS

In this course we will be using the Robot Operating System (ROS). ROS is a middleware platform that provides libraries and tools that help software developers create robot applications. It simplifies the integration of different components on the same system and over the network.

## The practical part of the assignment

Now we will start with the practical part of the assignment.

### What we use in this course

* Ubuntu __18.04__
* ROS __Melodic__
* Python
  * Version 2.7
  * Default with ROS Melodic

If you are interested you can read more [here](http://www.ros.org/reps/rep-0003.html).

### Mini-project

#### Installation

```bash
cd ~/catkin_ws/
wstool init src
cd ~/catkin_ws/src
wstool set -y irob_assignment_1 --git https://github.com/danielduberg/irob_assignment_1.git -v master
wstool set -y hector_slam --git https://github.com/tu-darmstadt-ros-pkg/hector_slam.git -v melodic-devel
wstool update
cd ~/catkin_ws
# This makes sure we compile in release mode (which means that the compiler optimizes the code)
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
source ~/.bashrc
```

#### Description

You will now do a mini-project where you should help a [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robot explore an unknown environment. The robot is called Burger and you can see a picture of Burger below.

![TurtleBot3 Burger](images/turtlebot3_burger.png "TurtleBot3 Burger. Image taken from: http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#data-of-turtlebot3-burger")

Image taken from: [http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#data-of-turtlebot3-burger](http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#data-of-turtlebot3-burger)

The ability to perform autonomous exploration is essential for an autonomous system operating in unstructured or unknown environments where it is hard or even impossible to describe the environment beforehand.

#### System description

Luckily for you, you do not have to solve the whole exploration task. Instead you will use an _exploration_ node, based on [receding horizon "next-best-view" (RH-NBV)](https://ieeexplore.ieee.org/abstract/document/7487281), that we have prepared for you. You will also make use of a _collision avoidance_ node, based on [the obstacle-restriction method (ORM)](https://ieeexplore.ieee.org/abstract/document/1545546) and [pure pursuit](https://apps.dtic.mil/docs/citations/ADA255524) (for smooth control), that ensures safe path following. In order for Burger to localize herself/himself you will also run a _SLAM_ node, based on [Hector SLAM](https://wiki.ros.org/hector_slam), which does mapping and localization. We will also make use of a [_costmap_2d_](https://wiki.ros.org/costmap_2d) node in order to make the exploration and collision avoidance simpler. Since you are not given a real Burger robot we will do this in simulation. The simulator we use is called [Gazebo](http://gazebosim.org/) and it is a popular simulator when working with ROS. Lastly, we will use a [_robot_state_publisher_](https://wiki.ros.org/robot_state_publisher) node to get the necessary transformations.

What you will have to do is create a _controller_ node that is using the exploration node and the collision avoidance node in order to move Burger around in the environment. This is one of the best things about ROS. That you do not have to do everything yourself. Instead you can use what others have done and simply "glue" the pieces together to make your desired system work.

#### Let's start

Open three terminals.

In the first terminal you should start the ROS Master:

```bash
roscore
```

In the second terminal you should launch the file `simulator.launch` inside `irob_assignment_1/launch` like this:

```bash
roslaunch irob_assignment_1 simulator.launch
```

And in the third terminal launch the file `start.launch` inside `irob_assignment_1/launch` like this:

```bash
roslaunch irob_assignment_1 start.launch
```

You will see a window called [RViz](https://wiki.ros.org/rviz) open:

![RViz view](images/rviz.png "RViz view")

In the main view of the RViz window you can see a small Turtlebot3 Burger robot in the middle of the white area. The white area of the map is called _free space_, it is space where the robot knows there is nothing. The large gray area is _unknown space_, it is space that the robot knowns nothing about. It can be either _free space_ or _occupied space_. _Occupied space_ is the cerise colored space. The cyan colored space is called _C-space_, it is space that are a distance from the _occupied space_ such that the robot would collied with the _occupied space_ if it would move into it. Take a look at the image below if you are interested. You can read more about it [here](https://wiki.ros.org/costmap_2d). Your job will be to help Burger explore as much of the _unknown space_ as possible.

![Costmap](https://wiki.ros.org/costmap_2d?action=AttachFile&do=get&target=costmapspec.png "Costmap")

Image taken from: [https://wiki.ros.org/costmap_2d](https://wiki.ros.org/costmap_2d)

If you open up [RQT](https://wiki.ros.org/rqt):

```bash
rqt
```

Then in the topbar select `Plugins->Introspection->Node Graph` and uncheck `Leaf topics`, you will see something like this:

![Node graph](images/rosgraph.png "Node graph")

Here you can see all of the nodes that were started when you ran the `roslaunch` commands before, and that we talked about previously. There are two nodes of interest for you here. The `/explore` node and the `/collision_avoidance` node.

The `/explore` node is providing an action server called `get_next_goal` with the type `irob_assignment_1/GetNextGoalAction`. If we take a look at the action definition:

```bash
# Goal definition
---
# Result definition
float64 gain
nav_msgs/Path path
---
# Feedback definition
float64 gain
nav_msgs/Path path
```

We see that in the request it does not require anything. So when you call this action server, you do not have to supply any arguments. The result you will get from the action server is a `gain` value and a `path`. The `gain` value tells you how valuable the `path` is. It is often how much new space you will discover when moving along the corresponding path.

In the feedback you can see that you, once again, get `gain` and a `path`. The longer the exploration algorithm is running the better path -- one with higher gain -- it will find. However, it is not always worth the time it takes to find the best path. So since you get a path with an associated gain, you might want to stop the exploration once you get a path with a `gain` higher than a certain value. You are not require to do this to pass this assignment, it is optional.

Okay, so lets say you have now called the explore action server and gotten a path. Now you want the robot to move along the path. You also do not want the robot to crash while following the path. Therefore we should now take a look at the `collision avoidance` node.

The collision avoidance node is providing a service called `get_setpoint` of type `irob_assignment_1/GetSetpoint`. If you type:

```bash
rossrv show irob_assignment_1/GetSetpoint
```

You will see the request and response messages:

```bash
# Define the request
nav_msgs/Path path
---
# Define the response
geometry_msgs/PointStamped setpoint
nav_msgs/Path new_path
```

You can see that the service wants a path, which is perfect because that is exactly what you got from the exploration action server. In return you will get a setpoint of type `geometry_msgs/PointStamped` and a new path. You get a new path since the collision avoidance node is removing points along the path which is thinks you have already moved passed. The `setpoint` is where you should move the robot next to follow the path in a safe and efficient way.

If we take a look at the setpoint message:

```bash
rosmsg show geometry_msgs/PointStamped
```

We see:

```bash
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z
```

The `header` tells you in which frame the `point` is in. If you print the frame:

```python
rospy.loginfo("The frame is: %s" setpoint.header.frame_id)
```

You will see that the point is specified in the X (where X is the frame you get) frame.

Okay, so now you have the point and you know which frame it is in. How do we make the robot move?

If we take a look at the topics list:

```bash
rostopic list
```

You should be able to find a topic called `/cmd_vel`. It sounds interesting, maybe it means "command velociy"? We want more info about the topic so we write:

```bash
rostopic info /cmd_vel
```

And it will output something like:

```bash
Type: geometry_msgs/Twist

Publishers: None

Subscribers: 
 * /gazebo (http://X:YYYYY/)
```

Here we see that there is one subscriber to the topic and no publisher. We also see that the message type that is communicated over the topic is called `geometry_msgs/Twist`. We take a look at the message definition:

```bash
rosmsg show geometry_msgs/Twist
```

And we get:

```bash
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

So the `geometry_msgs/Twist` message consist of a linear and angular velocity. Burger is a differential drive robot. Meaning Burger can only move forward/backward and rotate. To move forward `linear.x` should be positive. To move backwards it would be negative. To rotate you change the value of "angular.z", then the robot will be "yawing", rotating around the z-axis. Changing `linear.y, linear.z, angular.x, and angular.y` has no effect on the robot.

We can also see that the `geometry_msgs/Twist` message does not contain a `header`, therefore the subscriber to the `/cmd_vel` topic has no idea in what frame of reference the incoming velocity command has. Instead the subscriber assumes that the velocity command is in the robots frame.

Take a look at the TF tree by running the command `rqt`, to start RQT, then in the top bar select `Pluings->Visualization->TF Tree`:

![TF tree](images/frames.png "TF tree")

The robots frame is often called `base_link` and it is in this mini-project as well. So this is the frame the subscriber on the `/cmd_vel` topic is expecting the velocity command to be specified in. So if the setpoint you got from the `get_setpoint` service is not in the `base_link` frame then you have to transform it using TF2 to `base_link`.

After you have transformed the setpoint to `base_link` you should now convert the setpoint from a `geometry_msgs/PointStamped` message to a `geometry_msgs/Twist` message and publish it on the `/cmd_vel` topic. If you did the TF2 tutorial on writing a [tf2 listener](https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29) you should have a good idea about how to convert the `geometry_msgs/PointStamped` to a `geometry_msgs/Twist`. __Remember__ that you should transform the setpoint, so you cannot do exactly as it says in the TF2 tutorial. It is a good idea to use the function:

```python
transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)
```

When transforming the setpoint using the transform you got from:

```python
transform = tf_buffer.lookup_transform(...)
```

It can be a good idea to limit the linear and angular velocities, otherwise the robot will act strange. These seem to be good values:

```python
max_linear_velocity = 0.5
max_angular_velocity = 1.0
```

But you are free to try other values.

Then after that you should again call the `get_setpoint` with the `new_path` you got from the service the last time you called it, and then you do the same thing to transform and publish the new setpoint. You do this until the `new_path` does not contain any poses:

```python
while new_path.poses:
```

When that happens you should call the explore action server again to get a new path, and do everything over again until the action server returns an empty path and/or the gain is 0, meaning there is nothing new to explore.

It can be a good idea to limit the frequency of publishing to `/cmd_vel` to somewhere between 10 to 20. You can do that using the `rospy.Rate(X)` and then `rate.sleep()` as you did in the tutorials.

You can do the assignment using two different approaches:

#### Simple approach

If you go into the folder `irob_assignment_1/scripts` you will see a file called `controller.py`. Here we have made a skeleton for the controller node that you should write. So create your controller node in that file. When you are done you can test your controller by running:

```bash
rosrun irob_assignment_1 controller.py
```

If your code does not work yet, or you want to restart you simply have to close down your node and `start.launch` by going to the terminal where you started `start.launch` and press `CTRL+C`. Thereafter you launch the `start.launch` again:

```bash
roslaunch irob_assignment_1 start.launch
```

Together with your node in a seperate terminal:

```bash
rosrun irob_assignment_1 controller.py
```

Note that you do __not__ have to restart `roscore` or `simulator.launch`.

__Pseudocode__ for the assignment:

```python
# Init stuff
while True:
    path, gain = get_path_from_action_server()
    if path is empty:
        exit() # Done
    while path is not empty:
      path, setpoint = get_updated_path_and_setpoint_from_service(path)
      setpoint_transformed = transform_setpoint_to_robot_frame(setpoint)
      publish(setpoint_transformed)
      sleep()
```

You should be able to implement this in around __80-100 lines of Python code__. You should not need to import anything else or declare any other global variables then the once we provided in the skeleton. Of course you are allowed to do that if you want.

Sometimes you will see the robot getting stuck in walls. It is because the collision avoidance system was developed for a holonomic robot. To prevent this from happening it can be a good idea to set the linear velocity to 0 when Burger has to do a lot of turning. For example when the angular velocity is equal (or almost equal) to the maximum angular velocity you chose (1.0 if you want the same value as I used). This ensures that Burger follows the path more closely.

What you should see if you have done everything correct:

[![Finished mini-project](http://img.youtube.com/vi/L-Q_F20LVxU/0.jpg)](https://www.youtube.com/watch?v=L-Q_F20LVxU)

#### OPTIONAL: Callback based approach

If you did the mini-project using the simple approach you will notice that the exploration is quite slow and that Burger is just standing still a lot. This is because we are not using Actionlib to it's full potential. We are simply using the action server as a service.

To utilize Actionlib fully we have to use the _callback based action client_. Sadly, there is not a tutorial for a callback based action client written in Python on the Actionlib tutorial page. Therefore I will give you a minimal example of __Callback Based SimpleActionClient__ written in Python here:

```python
#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg

goal_client = None

def goal_active():
    rospy.loginfo("I got activated")


def goal_feedback(feedback):
    rospy.loginfo("I got feedback")

    # TODO: Do your stuff


def goal_result(state, result):
    if actionlib.TerminalState.SUCCEEDED == state:
        rospy.loginfo("Action returned succeeded")

        # TODO: Do your stuff

    elif actionlib.TerminalState.RECALLED == state:
        rospy.loginfo("Action returned recalled")
    elif actionlib.TerminalState.REJECTED == state:
        rospy.loginfo("Action returned rejected")
    elif actionlib.TerminalState.PREEMPTED = state:
        rospy.loginfo("Action returned preempted")
    elif actionlib.TerminalState.ABORTED = state:
        rospy.loginfo("Action returned aborted")
    elif actionlib.TerminalState.LOST = state:
        rospy.loginfo("Action returned lost")


def get_path():
    global goal_client
    goal_client.wait_for_server()
    goal = irob_assignment_1.msg.GetNextGoalGoal()
    goal_client.send_goal(goal, active_cb=goal_active,
                          feedback_cb=goal_feedback, done_cb=goal_result)


if __name__ == "__main__":
    rospy.init_node("controller")

    # Action client
    goal_client = actionlib.SimpleActionClient(
        "get_next_goal", irob_assignment_1.msg.GetNextGoalAction)

    get_path()

    rospy.spin()
```

The difference from the none callback based action client is that we specify three callback functions when calling send_goal:

* `active_cb`: This will be called as soon as the goal has been sent to the server.
* `feedback_cb`: This will be called every time the server sends feedback to the client.
* `done_cb`: This will be called when the server is done with the goal, when the client cancel the goal, or an error happens during processing of the goal.

As you can see we also do not `wait_for_result()` or `get_result()` since we will get the feedback or result as soon as it is available in the `feedback_cb` or `done_cb` callback, respectively.

You can find a code skeleton for the callback based approach in file controller_feedback.py in the folder `irob_assignment_/scripts`. To run it you type in the terminal:

```bash
rosrun irob_assignment_1 controller_feedback.py
```

How can you use the __Callback Based SimpleActionClient__ in order to increase the speed of the exploration? You can implement the assignment with __Callback Based SimpleActionClient__ in __less than 100 lines of Python code__.

What you should see if you have done everything correct:

[![Finished feedback mini-project](http://img.youtube.com/vi/m3NzMTdnsZU/0.jpg)](https://www.youtube.com/watch?v=m3NzMTdnsZU)
