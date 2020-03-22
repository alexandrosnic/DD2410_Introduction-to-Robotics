# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse


##################### LIBRARIES I IMPORTED #########################
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Empty, SetBool, SetBoolRequest 
####################################################################


# Tuck arm
class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        #rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        pt.common.Status.RUNNING

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        #rospy.loginfo("You reached tuckarm.")

        # self.goal = PlayMotionGoal()
        # self.goal.motion_name = 'home'
        # self.goal.skip_planning = True

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING



# Move head
class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raises the head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        #rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        #rospy.loginfo("You reached movehead.")

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING



# Pick cube
class pick(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        #rospy.loginfo("Initialising pick cube behaviour.")

        # Initialize, call and wait for the pick service
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv') 
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
        


        # execution checker
        self.picking = False
        self.finished = False

        # become a behaviour
        super(pick, self).__init__("Pick!")

    def update(self):


        # already picked the cube
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to pick the cube if haven't already
        elif not self.picking:

            # send the goal
            self.picking = True
            self.pick_req = self.pick_srv(True)

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.pick_req.success == True:

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_req.success:
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING


# Counter
class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        #rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS

# Go
class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        #rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular
        rospy.sleep(1)

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        #rospy.loginfo("Gooooooooooooo")
        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

# Place
class place(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        #rospy.loginfo("Initialising place cube behaviour.")

        # Initialize, call and wait for the pick service
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv') 
        rospy.wait_for_service(self.place_srv_nm, timeout=30)
        self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)


        # execution checker
        self.placing = False
        self.finished = False

        # become a behaviour
        super(place, self).__init__("Place!")

    def update(self):      

        # already picked the cube
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to pick the cube if haven't already
        elif not self.placing:

            # send the goal
            self.placing = True
            self.place_req = self.place_srv(True)

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.place_req.success:

            # than I'm finished!
            self.finished = True
            rospy.loginfo("1")
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_req.success:
            rospy.loginfo("2")
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            
            return pt.common.Status.RUNNING


class verifyPlacement(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name):

        #rospy.loginfo("Initialising cube detection.")
        self.cube_wrong = True
        
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        
        # become a behaviour
        super(verifyPlacement, self).__init__(name)


    def update(self):

        rospy.loginfo("3")

        self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.aruco_pose_cb)
        rospy.loginfo("4")
        if self.cube_wrong == True:
            rospy.loginfo("It failed to place the cube correctly")
            return pt.common.Status.FAILURE
            
        else:
            rospy.loginfo("It succeeded to place the cube correctly")
            return pt.common.Status.SUCCESS
            

    def aruco_pose_cb(self, aruco_pose_msg):
        self.aruco_pose = aruco_pose_msg
        self.cube_wrong = False

        # send the message
        #rate = rospy.Rate(10)
        #self.cmd_vel_pub.publish(self.move_msg)
        #rospy.loginfo('the marker position is yohooooo')
        #rate.sleep()

        # tell the tree that you're running
        #return pt.common.Status.FAILURE


# Tuck arm
class pregrasp(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        #rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        pt.common.Status.RUNNING

        # become a behaviour
        super(pregrasp, self).__init__("Pregrasp!")

    def update(self):

        #rospy.loginfo("You reached tuckarm.")

        # self.goal = PlayMotionGoal()
        # self.goal.motion_name = 'home'
        # self.goal.skip_planning = True

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING








# class counter(pt.behaviour.Behaviour):

#     """
#     Returns running for n ticks and success thereafter.
#     """

#     def __init__(self, n, name):

#         rospy.loginfo("Initialising counter behaviour.")

#         # counter
#         self.i = 0
#         self.n = n

#         # become a behaviour
#         super(counter, self).__init__(name)

#     def update(self):

#         # increment i
#         self.i += 1

#         # succeed after count is done
#         return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS



# class go(pt.behaviour.Behaviour):

#     """
#     Returns running and commands a velocity indefinitely.
#     """

#     def __init__(self, name, linear, angular):

#         rospy.loginfo("Initialising go behaviour.")

#         # action space
#         #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
#         self.cmd_vel_top = "/key_vel"
#         #rospy.loginfo(self.cmd_vel_top)
#         self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

#         # command
#         self.move_msg = Twist()
#         self.move_msg.linear.x = linear
#         self.move_msg.angular.z = angular

#         # become a behaviour
#         super(go, self).__init__(name)

#     def update(self):

#         # send the message
#         rate = rospy.Rate(10)
#         self.cmd_vel_pub.publish(self.move_msg)
#         rate.sleep()

#         # tell the tree that you're running
#         return pt.common.Status.RUNNING

# class tuckarm(pt.behaviour.Behaviour):

#     """
#     Sends a goal to the tuck arm action server.
#     Returns running whilst awaiting the result,
#     success if the action was succesful, and v.v..
#     """

#     def __init__(self):

#         rospy.loginfo("Initialising tuck arm behaviour.")

#         # Set up action client
#         self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

#         # personal goal setting
#         self.goal = PlayMotionGoal()
#         self.goal.motion_name = 'home'
#         self.goal.skip_planning = True

#         # execution checker
#         self.sent_goal = False
#         self.finished = False

#         # become a behaviour
#         super(tuckarm, self).__init__("Tuck arm!")

#     def update(self):

#         # already tucked the arm
#         if self.finished: 
#             return pt.common.Status.SUCCESS
        
#         # command to tuck arm if haven't already
#         elif not self.sent_goal:

#             # send the goal
#             self.play_motion_ac.send_goal(self.goal)
#             self.sent_goal = True

#             # tell the tree you're running
#             return pt.common.Status.RUNNING

#         # if I was succesful! :)))))))))
#         elif self.play_motion_ac.get_result():

#             # than I'm finished!
#             self.finished = True
#             return pt.common.Status.SUCCESS

#         # if failed
#         elif not self.play_motion_ac.get_result():
#             return pt.common.Status.FAILURE

#         # if I'm still trying :|
#         else:
#             return pt.common.Status.RUNNING

# class movehead(pt.behaviour.Behaviour):

#     """
#     Lowers or raisesthe head of the robot.
#     Returns running whilst awaiting the result,
#     success if the action was succesful, and v.v..
#     """

#     def __init__(self, direction):

#         rospy.loginfo("Initialising move head behaviour.")

#         # server
#         mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
#         self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
#         rospy.wait_for_service(mv_head_srv_nm, timeout=30)

#         # head movement direction; "down" or "up"
#         self.direction = direction

#         # execution checker
#         self.tried = False
#         self.done = False

#         # become a behaviour
#         super(movehead, self).__init__("Lower head!")

#     def update(self):

#         # success if done
#         if self.done:
#             return pt.common.Status.SUCCESS

#         # try if not tried
#         elif not self.tried:

#             # command
#             self.move_head_req = self.move_head_srv(self.direction)
#             self.tried = True

#             # tell the tree you're running
#             return pt.common.Status.RUNNING

#         # if succesful
#         elif self.move_head_req.success:
#             self.done = True
#             return pt.common.Status.SUCCESS

#         # if failed
#         elif not self.move_head_req.success:
#             return pt.common.Status.FAILURE

#         # if still trying
#         else:
#             return pt.common.Status.RUNNING