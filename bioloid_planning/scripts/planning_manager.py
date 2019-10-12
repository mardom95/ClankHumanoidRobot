#!/usr/bin/env python

#utils
import rospy
import threading
import sys
import copy
import numpy
import math
import json
import tf

#Moveit
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import PickupAction, PickupGoal, PickupResult
from moveit_msgs.msg import PlaceAction, PlaceGoal
from moveit_msgs.msg import *
from moveit_msgs.msg import MoveItErrorCodes, Constraints

from geometry_msgs.msg import Pose, Point, PoseStamped, PoseArray, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint

#Planning messages
from tf.transformations import quaternion_from_euler
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import GetOctomap

# Main class
class PlanningManager:
    def __init__(self):

        # Retrieve params:
        self._robot_group_name     = rospy.get_param('~robot_gorup_name', 'robot')

        # Create planning scene and robot commander:
        self._scene = PlanningSceneInterface()
        self._robot = RobotCommander()

        # Retrieve groups (arm and gripper):
        self._robot_group     = self._robot.get_group(self._robot_group_name)
        rospy.logwarn('Robot group retrieved')

        self._robot_group.set_named_target('default_home')
        self._robot_group.go()

        self._robot_group.set_named_target('sitting_chair')
        self._robot_group.go()

        self._robot_group.set_named_target('sitting_on_floor')
        self._robot_group.go()

        self._robot_group.set_named_target('default_home')
        self._robot_group.go()



def main():

    pMan = PlanningManager()

    rospy.spin()

if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('planning_client')

    main()

    roscpp_shutdown()

