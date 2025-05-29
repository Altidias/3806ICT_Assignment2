#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import threading
import time

class SLAMInitializer:
    def __init__(self):
        rospy.init_node('slam_initializer')
        
        self.robots = ['robot1', 'robot2', 'robot3']
        self.cmd_pubs = {}
        self.map_received = {}
        
        # publishers and subscribers for each robot
        for robot in self.robots:
            self.cmd_pubs[robot] = rospy.Publisher(f'/{robot}/cmd_vel', Twist, queue_size=1)
            self.map_received[robot] = False
            rospy.Subscriber(f'/{robot}/map', OccupancyGrid, 
                           lambda m, r=robot: self.map_callback(m, r))
        
        rospy.loginfo("SLAM Initializer ready")
        
    def map_callback(self, msg, robot):
        if not self.map_received[robot] and len(msg.data) > 100:
            self.map_received[robot] = True
            rospy.loginfo(f"✓ {robot} SLAM initialized! Map received.")
            
    def move_robot(self, robot, linear=0.0, angular=0.0, duration=1.0):
        if robot not in self.cmd_pubs:
            return
            
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_pubs[robot].publish(twist)
            rate.sleep()
            
        # stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pubs[robot].publish(twist)
        
    def initialize_robot(self, robot):
        if self.map_received[robot]:
            rospy.loginfo(f"{robot} already initialized")
            return
            
        rospy.loginfo(f"Initializing {robot}...")
        
        movements = [
            ("forward", 0.15, 0.0, 2.0),
            ("rotate left", 0.0, 0.4, 2.0),
            ("forward", 0.15, 0.0, 2.0),
            ("rotate right", 0.0, -0.4, 3.0),
            ("forward", 0.1, 0.0, 1.0),
        ]
        
        for desc, linear, angular, duration in movements:
            rospy.loginfo(f"  {robot}: {desc}")
            self.move_robot(robot, linear, angular, duration)
            
            if self.map_received[robot]:
                rospy.loginfo(f"  {robot}: SLAM started!")
                break
                
        if not self.map_received[robot]:
            rospy.logwarn(f"  {robot}: SLAM did not start.")
            
    def initialize_all(self):
        threads = []
        
        # check which robots exist
        existing_robots = []
        for robot in self.robots:
            try:
                # check if cmd_vel topic exists
                rospy.wait_for_message(f'/{robot}/cmd_vel', Twist, timeout=1.0)
                existing_robots.append(robot)
            except:
                pass
                
        if not existing_robots:
            rospy.logerr("No robots found")
            return
            
        rospy.loginfo(f"Found robots: {existing_robots}")
        
        for robot in existing_robots:
            thread = threading.Thread(target=self.initialize_robot, args=(robot,))
            thread.start()
            threads.append(thread)
            time.sleep(0.5)
            
        for thread in threads:
            thread.join()
            
        rospy.loginfo("\n=== INITIALIZATION COMPLETE ===")
            
def main():
    initializer = SLAMInitializer()
    initializer.initialize_all()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass