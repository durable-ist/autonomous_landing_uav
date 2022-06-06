import sys # argv, argc
import datetime
import rospy
import math
import numpy as np
# Libraries with the messages structures used by ros services and topics
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from mavros_msgs.msg import State
from geometry_msgs.msg import Pose, Twist, Vector3, Transform, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
# Libraries for the transformations between frames
import tf
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion
import tf2_ros
from tf2_ros import TransformListener
# Import other classes
from pid import PID


class Publisher(object):
    """ PUBLISHER / SUBSCRIBER """

    def __init__(self,ros_node):

        rosNode = ros_node

        ## Subscribers ##
        self.sub_marker = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback_marker)
        self.sub_State = rospy.Subscriber("/mavros/state", State, self.check_Remote_mode)

        #self.sub_odom = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.callback_odom)
        ## Publishers ##
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        ## Iris Variables ##

        self.obs = []
        self.hasObs = False
        self.firstObs = False
        self.current_state = State()
        self.prev_state = State()

        self.rate = rospy.Rate(10)

        #Tramsforms
        self.listener = tf.TransformListener()

        ## Classes ## 
        self.pid = PID()

    def start(self):

        while( not self.check_landing()):

            if(self.firstObs == True and self.current_state.mode == "GUIDED") : 
                #print("Mode is: ", self.current_state.mode)
                target_angles = self.pid.target_pose(self.obs, self.hasObs) 
                print("Target angles: ", target_angles)
                self.vel(target_angles)
                self.prev_state = self.current_state
                self.rate.sleep()
            
            if (self.prev_state.mode != self.current_state.mode):
                # if flight mode changed, re initialize PID params
                print("PID Re_initialized")
                self.pid.prev_time = [0, 0, 0, 0]
                self.pid.prev_error = [0.0, 0.0, 0.0, 0.0]
                self.pid.error_i = [0.0, 0.0, 0.0, 0.0]
                self.pid.pid_distance = [0, 0, 0]
                self.prev_state = self.current_state
            
        
        return 'landed'
    
    def check_landing(self):

        # HOW TO KNOW IF IT LANDED
        
        z = 0.4
        if(z <= 0.3):
            return True

        return False
    
    def check_Remote_mode(self, state):
        #check to see if remote is in guided mode
        self.current_state=state

            
    """
    > Markers Callback
    """
    def callback_marker(self, msg):
        
        observations = np.zeros( (len(msg.transforms) , 4) )
        i = 0
        self.hasObs = False

        # Parse of the fiducial_transforms message
        for marker in msg.transforms:

            try:
                marker_id = "fiducial_T_" + str(marker.fiducial_id)
                (trans, rot) = self.listener.lookupTransform('base_stablized', marker_id, rospy.Time(0))
		#print("TRANS:", trans)

                # Store the measurements of each marker
                observations[i] = np.array([marker.fiducial_id,
                    trans[0],
                    trans[1],
                    -trans[2] ]) #The x is pointing downwards in the camera frame, therefore it's the z

                self.hasObs = True
                self.firstObs = True
                i+=1
            
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print ("Fail", e)
        

        # Obtain the average of all the measurements taken in this iteration
        if(i!=0):       # If matrix is not empty
            average_x = sum(observations[:,1]) / len(observations)
            average_y = sum(observations[:,2]) / len(observations)
            average_z = sum(observations[:,3]) / len(observations)    

            self.obs = np.array([average_x,
                                average_y,
                                average_z])
            

    def vel(self,target_angles):
    
        msg_iris = Twist()
        msg_iris.linear.x = target_angles[0]
        msg_iris.linear.y = target_angles[1]
        msg_iris.linear.z = - target_angles[2]

        self.pub_vel.publish(msg_iris)

        
