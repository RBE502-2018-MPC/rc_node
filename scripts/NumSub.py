#!/usr/bin/env python
import roslib
roslib.load_manifest('py_test')
import rospy
import sys, time, os
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped


class NumSub:


    def __init__(self):	
        self.num_sub = rospy.Subscriber("/vrpn_client_node/rc_car/pose", PoseStamped, self.callback, queue_size = 1)	
        self.coordinates = np.matrix([0.0, 0.0, 0.0])
		
    def callback(self,data):
    	pos = data.pose.position
        self.coordinates = np.append(self.coordinates, np.matrix([pos.x, pos.y, pos.z]), axis=0)
        np.savetxt("src/py_test/data.csv", self.coordinates, delimiter=",")
    	
					
def main(args):  
    rospy.init_node('NumSub', anonymous=True)
    ns = NumSub()	
    try:
	rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
