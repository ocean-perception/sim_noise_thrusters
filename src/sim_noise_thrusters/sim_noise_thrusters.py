#!/usr/bin/env python

# Basic ros
import rospy

# Import msgs
from cola2_msgs.msg import Setpoints

# Import random for creating an anomaly onte the thrusters
import random

class ThrusterNoiseGenerator:
    def __init__(self):
        # init
        rospy.init_node('sim_noise_thrusters')
        self.pub = rospy.Publisher('setpoint_pub', Setpoints, queue_size=1)
        self.sub = rospy.Subscriber("setpoint_sub", Setpoints, self.setpointsCallback)
        self.start_time = rospy.Time.now()
        rospy.spin()

    def setpointsCallback(self, msg):
        new_msg = Setpoints()
        new_msg.header = msg.header
        new_msg.setpoints = [0, 0, 0]

        current_time = rospy.Time.now()

        diff = current_time - self.start_time # time - time = duration

        diff_sec = diff.to_sec() # floating point 

        randomness = [0, 0, 0]

        if diff_sec > 10*60: # frequncy of fault occurance every 10min
            randomness[0] = round(random.uniform(-1, 1), 3)
            randomness[1] = round(random.uniform(-1, 1), 3)
            randomness[2] = round(random.uniform(-1, 1), 3)
            
        new_msg.setpoints[0] = msg.setpoints[0] + randomness[0]
        new_msg.setpoints[1] = msg.setpoints[1] + randomness[1]
        new_msg.setpoints[2] = msg.setpoints[2] + randomness[2]
    
        self.pub.publish(new_msg)

if __name__ == '__main__':
    ThrusterNoiseGenerator()

    
