#!/usr/bin/env python

"""
Navigation node outputs waypoint goals for the robot using odometry, scene info and intent.
"""

import os
import rospy
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from intent_recognizer.msg import Intent
from scene_understanding.msg import Scene
# from navigation_manager.msg import Scene, Intent

UPDATE_RATE = 5 # [Hz]

class NavManager(object):
    """
    Computes waypoint goals from POI detections.
    """
    def __init__(self):
        
        # most recent pose of robot from odometry
        self.recent_pose = None
        # recent scene info
        self.scene_info = None
        # recent intent info
        self.intent_info = None

        # current goal of robot
        self.current_goal = None

        # create node, publishers, and subscribers
        rospy.init_node('nav_manager')
        
        self.pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)

        rospy.Subscriber('/scene_info',Scene,self.scene_info_cb)
        rospy.Subscriber('/movo/odometry/local_filtered',Odometry,self.odometry_cb)
        rospy.Subscriber('/intent',Intent,self.intent_cb)

        rate = rospy.Duration(UPDATE_RATE)
        while not rospy.is_shutdown():

            if self.current_goal is None:
                
                # make sure there is all necessary info to compute goal
                if (((self.recent_pose is not None) and (self.scene_info is not None)) and self.intent_info is not None):

                    # get goal and generate waypoint goal message
                    self.current_goal = self.compute_goal(self.recent_pose,self.scene_info,self.intent_info)
                    msg = self.gen_goal_msg(self.current_goal)

                    # publish message
                    self.pub.publish(msg)

            rospy.sleep(rate)

    def compute_goal(self,pose,scene_info,intent):
        """
        Computes waypoint goal for robot to travel to.

        Inputs:

            pose -- recent pose of robot (position and ortientation)
            scene_info -- object detections from scene
            intent -- recognized intent of user

        Returns:

            goal -- [x,y] goal point of robot
        """

        # establish if poi is known
        if intent.poi_present:

            # get dpeth and angle in image frame of poi
            depth = intent.poi_depth
            angle = np.deg2rad(intent.poi_angle)

            # compute new goal using current position and poi data
            goal_x = pose.position.x + depth*np.cos(angle)
            goal_y = pose.position.y + depth*np.sin(angle)

        else:

            goal_x = pose.position.x + intent.forward_distance
            goal_y = pose.position.y

        return [goal_x,goal_y]

    def gen_goal_msg(self,goal):
        """
        Generates waypoint goal message to be published.

        Inputs:

            goal -- [x,y] goal coordinates

        Returns:

            msg -- generated goal message (PoseStamped)
        """
        msg = PoseStamped()
        
        msg.header.frame_id = '/map'

        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]

        return msg

    def odometry_cb(self,msg):
        """
        Saves most recent odometry message.
        """
        self.recent_pose = msg.pose.pose

    def scene_info_cb(self,msg):
        """
        Saves most recent scene info message from SceneUnderstanding module.
        """
        self.scene_info = msg

    def intent_cb(self,msg):
        """
        Saves most recent user intent message from IntentRecognizer module.
        """
        self.intent_info = msg



if __name__ == "__main__":
    NavManager()

    
