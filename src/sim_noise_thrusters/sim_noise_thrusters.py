#!/usr/bin/env python

# Basic ros
import rospy

# Import msgs
from cola2_msgs.msg import Setpoints

class ThrusterNoiseGenerator:
    def __init__(self):
        # init
        rospy.init_node('sim_noise_thrusters')
        self.pub = rospy.Publisher('setpoint_pub', Setpoints, queue_size=1)
        self.sub = rospy.Subscriber("setpoint_sub", Setpoints, self.setpointsCallback)
        rospy.spin()

    def setpointsCallback(self, msg):
        new_msg = Setpoints()
        new_msg.header = msg.header
        new_msg.setpoints = [0, 0, 0]
        new_msg.setpoints[0] = msg.setpoints[0]
        new_msg.setpoints[1] = msg.setpoints[1] + 0.1 # Example addin 0.1 to setpoint
        new_msg.setpoints[2] = msg.setpoints[2]
        self.pub.publish(new_msg)

if __name__ == '__main__':
    ThrusterNoiseGenerator()

    
