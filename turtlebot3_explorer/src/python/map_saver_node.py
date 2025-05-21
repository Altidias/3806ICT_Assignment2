#!/usr/bin/env python3

import rospy
import subprocess
import os
from std_srvs.srv import Empty, EmptyResponse

class MapSaverNode:
    def __init__(self):
        rospy.init_node('map_saver_node')
        
        self.map_save_path = rospy.get_param('~map_save_path', os.path.expanduser('~/map'))

        self.save_service = rospy.Service('save_map', Empty, self.save_map_callback)
        
        
        
    def save_map_callback(self, req):
        try:
            subprocess.call(["rosrun", "map_server", "map_saver", "-f", self.map_save_path])
            rospy.loginfo("Map saved")
            return EmptyResponse()
        except Exception as e:
            rospy.logerr(f"Failed: {e}")
            return EmptyResponse()
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        saver = MapSaverNode()
        saver.run()
    except rospy.ROSInterruptException:
        pass