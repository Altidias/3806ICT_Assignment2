#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

def trigger_rescue():
    rospy.init_node('trigger_rescue')
    
    # Wait for system to initialize
    rospy.sleep(5.0)
    
    # Publish exploration complete
    pub = rospy.Publisher('/exploration_complete', Bool, queue_size=1, latch=True)
    rospy.sleep(1.0)  # Wait for publisher to connect
    
    msg = Bool()
    msg.data = True
    pub.publish(msg)
    
    rospy.loginfo("Triggered exploration complete - rescue should start!")
    rospy.sleep(1.0)  # Keep node alive briefly to ensure message is sent

if __name__ == '__main__':
    trigger_rescue()