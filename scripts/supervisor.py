#!/usr/bin/env python

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Int16, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject, RescueMessage
import tf
import math
from enum import Enum

# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 5 #sec

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .3
ANIMAL_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 5
DIST_THRESHOLD = 0.5

# max number of attempts to get to a desired pose
MAX_NAV_PATH_ATTEMPTS = 50

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6
    ANIMAL_STOP = 7
    ANIMAL_CROSS = 8

class High_Mode(Enum):
    EXPLORE = 1
    RESCUE = 2
    COMPLETE = 3 #Back Home

class Supervisor:
    """ the state machine of the turtlebot """

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        # current pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # pose goal
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        # current mode
        self.mode = Mode.IDLE
        self.high_mode = High_Mode.EXPLORE
        self.last_mode_printed = None
        self.last_high_mode_printed = None
        self.animal_poses = {}

        # EXPLORATION - MOD
        #self.exploration_points = []
        #self.exploration_points = [np.array([3.354,0.314,-3.104]), np.array([2.5,0.256,-3.124]), np.array([2.33,1.47,1.586]), np.array([1.375, 1.496,0]), np.array([0.252, 1.515, 0]), np.array([0.291, 0.331,-1.524]), np.array([2.4, 2.735,3.086]),np.array([1.612, 2.811,-3.104]),np.array([3.03, 1.474,0.023]) ]

        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        rospy.Subscriber('/detector/cat', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/dog', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/elephant', DetectedObject, self.animal_detected_callback)
        rospy.Subscriber('/detector/bear', DetectedObject, self.animal_detected_callback)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        #keyboard teleop functionalities
        rospy.Subscriber('/cmd_teleop_vel', Twist, self.teleop_vel_callback)
        rospy.Subscriber('/cmd_teleop_mode', Int16, self.teleop_mode_callback)

        #Modified for navigator robustness
        rospy.Subscriber('/cmd_path_fail', Bool, self.nav_path_fail_callback)
        self.last_pose = self.x, self.y, self.theta
        self.nav_path_fail_count = 0

        self.trans_listener = tf.TransformListener()

        # Rescue
        rospy.Subscriber('/Bob_to_tb3', Int16, self.rescue_animal)
        self.to_Bob_publisher = rospy.Publisher("/tb3_to_Bob", String, queue_size=10)
        self.menu_published = False
        self.rescue_end = None


    #Modified for navigator robustness
    def nav_path_fail_callback(self, msg):
        if msg.data == True:
            self.nav_path_fail_count += 1
        else:
            self.nav_path_fail_count = 0


    # MODIFIED
    def teleop_vel_callback(self, msg):
        # rospy.loginfo("TELEOP CMD RECEIVED")
        self.mode = Mode.MANUAL
        self.cmd_vel_publisher.publish(msg)

    # MODIFIED
    def teleop_mode_callback(self, msg):
        if msg.data == 8:
            self.high_mode = High_Mode.EXPLORE
            return
        elif msg.data == 9:
            #rospy.loginfo("rescue callback")
            self.high_mode = High_Mode.RESCUE
            return
        if msg.data == 0:
            #rospy.loginfo("rescue callback")
            self.high_mode = High_Mode.COMPLETE
            return

        for mode in Mode:
            if mode.value == msg.data:
                self.mode = mode

    def rescue_animal(self, msg):
        if self.high_mode == High_Mode.RESCUE and self.mode == Mode.IDLE:
            if msg.data in self.animal_poses:
                pose  = self.animal_poses[msg.data]
                self.x_g, self.y_g, self.theta_g = pose[1]
                rospy.loginfo("Going to rescue: {} at {}".format(pose[0], pose[1]))
                del self.animal_poses[msg.data]
                self.mode = Mode.NAV
                self.menu_published = False
            else:
                rospy.loginfo("Invalid animal selected {}".format(msg.data))

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """

        self.x_g = msg.pose.position.x
        self.y_g = msg.pose.position.y
        rotation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(rotation)
        self.theta_g = euler[2]

        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if 0 < dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.init_stop_sign()

    def animal_detected_callback(self, msg):
        """ callback for when the detector has found an animal. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        #rospy.loginfo("Animal detected:{} at distace {} with confidence {}".format(msg.name, msg.distance, msg.confidence))

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if 0 < dist < ANIMAL_MIN_DIST and self.mode in [Mode.NAV, Mode.MANUAL]:
            if (self.high_mode == High_Mode.EXPLORE):
                self.init_animal_explore(msg.name)
            else:
                #self.init_animal_reached()
                pass

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired nav goal to the navigator """
        #rospy.loginfo("close to last pose: {} with fail count: {}".format(self.close_to(*self.last_pose), self.nav_path_fail_count))

        if self.close_to(*self.last_pose) and self.nav_path_fail_count > MAX_NAV_PATH_ATTEMPTS:
            # Failed to get to pose
            rospy.loginfo("Failed to navigate to desired pose. Back to IDLE")
            self.mode = Mode.IDLE
            self.nav_path_fail_count = 0
        else:
            # Go to pose
            nav_g_msg = Pose2D()
            nav_g_msg.x = self.x_g
            nav_g_msg.y = self.y_g
            nav_g_msg.theta = self.theta_g

            self.last_pose = self.x, self.y, self.theta
            #self.nav_path_fail_count = 0

            self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def init_animal_explore(self,cl):
        detected_cl = np.array([pose[1] for pose in self.animal_poses.values() if pose[0] == cl])
        #rospy.loginfo("Detected cl: {} with shape {}".format(detected_cl, detected_cl.shape))
        if detected_cl.shape[0] == 0:
            rospy.loginfo("{} number animal added for rescue: {}!".format(len(self.animal_poses)+1,cl))
            #rospy.loginfo("{} detection. Class: {}".format(detected_cl.shape[0]+1, cl))
            #rospy.loginfo("Robot position: {}".format([self.x, self.y]))
            self.animal_poses[len(self.animal_poses)] = (cl, np.array([self.x, self.y, self.theta]))
            if self.mode != Mode.MANUAL:
                self.init_crossing_animal()     #self.mode = Mode.ANIMAL_CROSS
        else:
            #rospy.loginfo("{} detection: Class: {}".format(detected_cl.shape[0], cl))
            #rospy.loginfo("Position: {}".format([self.x, self.y]))
            dist = np.linalg.norm(np.array([self.x, self.y]).reshape((1,2)) - detected_cl[:,0:2], axis = 1)
            if(np.min(dist) > DIST_THRESHOLD):
                rospy.loginfo("New {} number animal added for rescue: {}!".format(len(self.animal_poses)+1,cl))
                self.animal_poses[len(self.animal_poses)] = (cl, np.array([self.x, self.y, self.theta]))
                if self.mode != Mode.MANUAL:
                    self.init_crossing_animal() #self.mode = Mode.ANIMAL_CROSS
            else:
                rospy.loginfo("Same animal seen before: {}!".format(cl))

    def init_animal_reached(self):
        """ initiates a stop sign maneuver """

        self.animal_start = rospy.get_rostime()
        self.mode = Mode.ANIMAL_STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def has_stopped_animal(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.ANIMAL_STOP and (rospy.get_rostime()-self.animal_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def init_crossing_animal(self):
        """ initiates an intersection crossing maneuver """

        self.animal_cross_start = rospy.get_rostime()
        self.mode = Mode.ANIMAL_CROSS


    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))


    def has_crossed_animal(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.ANIMAL_CROSS and (rospy.get_rostime()-self.animal_cross_start)>rospy.Duration.from_sec(CROSSING_TIME))


    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        try:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        #original
        # if not(self.last_high_mode_printed == self.high_mode):
        #     rospy.loginfo("Current High Mode: %s", self.high_mode)
        #     self.last_high_mode_printed = self.high_mode

        #Back Home
        if not(self.last_high_mode_printed == self.high_mode):
            rospy.loginfo("Current High Mode: %s", self.high_mode)
            if(self.high_mode == High_Mode.RESCUE and self.rescue_end is None):
                self.rescue_end = np.array([self.x, self.y, self.theta])
            self.last_high_mode_printed = self.high_mode


        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            if self.high_mode == High_Mode.RESCUE and len(self.animal_poses) > 0 and not self.menu_published:
                menu = ""
                for k in sorted(self.animal_poses):
                    anm_class, anm_pose = self.animal_poses[k]
                    menu += "{}: {} at {} \n".format(k, anm_class, anm_pose)
                self.to_Bob_publisher.publish(menu)
                self.menu_published = True

            # EXPLORATION - MOD
            # elif (self.high_mode == High_Mode.EXPLORE and len(self.exploration_points)>0):
            #     current_pose = self.exploration_points[0]
            #     rospy.loginfo("Reached the Explore Automatic case {} ".format(current_pose))
            #     self.x_g, self.y_g, self.theta_g = current_pose
            #     del self.exploration_points[0]
            #     self.mode = Mode.NAV


            #Back home
            elif(self.high_mode == High_Mode.RESCUE and len(self.animal_poses) == 0):
                self.x_g, self.y_g, self.theta_g = self.rescue_end
                rospy.loginfo("Completed Rescue, going back to fire station")
                self.mode = Mode.NAV
                self.high_mode = High_Mode.COMPLETE


            else:
                self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.ANIMAL_STOP:
            # at animal pose
            if self.has_stopped_animal():
                #self.init_crossing_animal()
                self.mode = Mode.IDLE
            else:
                self.stay_idle()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()


        elif self.mode == Mode.ANIMAL_CROSS:
            # crossing an animal
            if self.has_crossed_animal():
                self.mode = Mode.NAV
            else:
                self.nav_to_pose()


        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                if self.high_mode == High_Mode.RESCUE:
                    self.init_animal_reached()
                else:
                    self.mode = Mode.IDLE
            else:
                self.nav_to_pose()

        elif self.mode == Mode.MANUAL:
            pass
            #self.x_g = self.x
            #self.y_g = self.y
            #self.theta_g = self.theta
            #self.nav_to_pose()

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
