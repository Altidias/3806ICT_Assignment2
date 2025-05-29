#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import threading
import time

class SLAMInit:
    def __init__(self):
        rospy.init_node('slam_initializer')
        
        # params
        self.robots = rospy.get_param('~robot_names', ['robot1', 'robot2'])
        self.moves = rospy.get_param('slam_initializer/initialization_moves', [
            {'description': 'forward', 'linear': 0.15, 'angular': 0.0, 'duration': 2.0},
            {'description': 'rotate_left', 'linear': 0.0, 'angular': 0.4, 'duration': 2.0},
            {'description': 'forward', 'linear': 0.15, 'angular': 0.0, 'duration': 2.0},
            {'description': 'rotate_right', 'linear': 0.0, 'angular': -0.4, 'duration': 3.0},
            {'description': 'forward', 'linear': 0.1, 'angular': 0.0, 'duration': 1.0}
        ])
        
        # state
        self.pubs = {}
        self.got_map = {}
        
        # setup pubs/subs
        for robot in self.robots:
            self.pubs[robot] = rospy.Publisher(f'/{robot}/cmd_vel', Twist, queue_size=1)
            self.got_map[robot] = False
            
            # sub to map
            rospy.Subscriber(f'/{robot}/map', OccupancyGrid, 
                           lambda msg, n=robot: self.map_cb(msg, n))
        
    def map_cb(self, msg, robot):
        """check if map started"""
        if not self.got_map[robot] and len(msg.data) > 100:
            # check for explored area
            known = sum(1 for c in msg.data if c != -1)
            if known > 50:
                self.got_map[robot] = True
                
    def move(self, robot, lin, ang, dur):
        """move robot"""
        if robot not in self.pubs:
            return
            
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        
        end = rospy.Time.now() + rospy.Duration(dur)
        rate = rospy.Rate(10)
        
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.pubs[robot].publish(cmd)
            rate.sleep()
            
        # stop
        self.pubs[robot].publish(Twist())
        
    def init_robot(self, robot):
        """init slam for robot"""
        if self.got_map[robot]:
            return
            
        # do moves
        for m in self.moves:
            if self.got_map[robot]:
                break
                
            self.move(robot, m['linear'], m['angular'], m['duration'])
            rospy.sleep(0.5)
            
    def init_all(self):
        """init all robots"""
        rospy.loginfo("=== STARTING SLAM INITIALIZATION ===")
        
        # start threads
        threads = []
        for robot in self.robots:
            rospy.loginfo(f"Starting SLAM init for {robot}")
            t = threading.Thread(target=self.init_robot, args=(robot,))
            threads.append(t)
            t.start()
            time.sleep(1.0)  # stagger
            
        # wait
        for t in threads:
            t.join()
        
        rospy.loginfo("=== SLAM INITIALIZATION COMPLETE ===")

def main():
    try:
        init = SLAMInit()
        
        # wait a bit
        rospy.sleep(2.0)
        
        # init robots
        init.init_all()
        
        # done
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()