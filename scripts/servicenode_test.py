#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from ragnar_teensy.srv import *
from std_msgs.msg import Float64MultiArray
 
servicetotest = 'set_stiffness_matrix'
def set_matrix():
    rospy.wait_for_service(servicetotest)
    try:
        
        servarg = Float64MultiArray()
        data = [None]*9
        for x in range(9):
          data[x] = x * 0.1
        servarg.data = data
        print(data)
        servicce = rospy.ServiceProxy(servicetotest, threeby3Matrix)
        servicce(servarg)
        
        return 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return

if __name__ == "__main__":
    

    print "Requesting "
    set_matrix()