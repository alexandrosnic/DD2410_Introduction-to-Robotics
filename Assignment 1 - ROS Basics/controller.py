#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, sqrt

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub
    while path.poses: 
        # Call service client with path
        response_service = control_client(path) # The service has one request: the path. 
        new_path = response_service.new_path # In exchange it gives you as response: setpoint and new path
        setpoint = response_service.setpoint

        # Transform Setpoint from service client
        transform = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id , rospy.Time()) # Create the transform matrix 
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform) # Assign the transform matrix to the setpoint

        # Create Twist message from the transformed Setpoint
        Twist_msg = Twist()

        # Assign the twist velocities as a direction vector toward the setpoint
        Twist_msg.angular.z = atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
        Twist_msg.linear.x = sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)

        # Clip maximum velocities
        # if(Twist_msg.linear.x >= 0):
        Twist_msg.linear.x = min(max_linear_velocity, Twist_msg.linear.x)
        '''    
        else: 
            Twist_msg.linear.x = max(-max_linear_velocity, Twist_msg.linear.x)
        
        if(Twist_msg.angular.z >= 0):
            Twist_msg.angular.z=min(max_angular_velocity, Twist_msg.angular.x)
        else: 
            Twist_msg.angular.z=max(-max_angular_velocity, Twist_msg.angular.x)
            '''
            
        # Publish Twist
        pub.publish(Twist_msg)
        rate.sleep()
		
        # Call service client again if the returned path is not empty and do stuff again
        path = new_path            

    # Send 0 control Twist to stop robot
    Twist_msg.angular.z = 0
    Twist_msg.linear.x = 0
    pub.publish(Twist_msg)
    rate.sleep()

    # Get new path from action server
    get_path()


def get_path():    
    global action_client

    # Get path from action server
    action_client.wait_for_server() # Wait for server
    get_goal = irob_assignment_1.msg.GetNextGoalGoal() # Creates a goal to send to the action server
    # print(get_goal)

    action_client.send_goal(get_goal) # Send the goal to the action server
    action_client.wait_for_result() # Wait for the server that is listening to the goals
    
    # Call move with path from action client
    move(action_client.get_result().path) 



if __name__ == "__main__":
    # Init node
    rospy.init_node("controller")
    rate = rospy.Rate(10.0) # A frequency of 10Hz - In pair with sleep command

    # Init publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #queue size of 10, just to be sure that not any message will be dropped

    # Init simple action client
    action_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    # Initialize buffer and listener as soon as possible
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
   
    # Init service client
    rospy.wait_for_service('get_setpoint')
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)

    # Call get_path def
    get_path()

    # Spin
    rospy.spin()