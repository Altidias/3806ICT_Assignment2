#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PoseArray, PoseStamped
from std_srvs.srv import Empty, EmptyResponse
from turtlebot3_explorer.msg import ExecutionStatus

class ExplorationCoordinator:
    """Coordinates the overall exploration process"""
    
    def __init__(self):
        
    
    def status_callback(self, msg):
        
    
    def frontiers_callback(self, msg):
        
    
    def check_exploration_status(self, event):
        
    
    def complete_exploration(self):
        

if __name__ == '__main__':
    try:
        coordinator = ExplorationCoordinator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass