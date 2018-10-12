#!/usr/bin/env python
'''
Node to write and read from the Serial teensy controlling the ragnar robot
'''
import rospy
import tf
import numpy as np
from math import sin, cos, pi
import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState 
from std_msgs.msg import String

from SerialDataGateway import SerialDataGateway

from ragnar_teensy.srv import *

class Teensy(object):
    '''
    Helper class for communicating with an Arduino board over serial port
    '''
    def _HandleReceivedLine(self, line):
        #print(line)
        #print(len(line))
        self._Counter = self._Counter + 1
        #self._speedsimupub.publish(String(str(self._Counter) + " " + line))
        if (len(line) > 0):
            lineParts = line.split('\t')
            #print(ord(lineParts[0][11]))
            #print("there")
            if (lineParts[0] == 'q'):
                #self._printThis(lineParts)
                self._JointsPublisher(lineParts)
                return
            if (lineParts[0] == 'o'):
                #self._BroadcastOdometry(lineParts)
                return
            if (lineParts[0] == "activate_me"):
                # rospy.loginfo("i am here in activation")
                # controller requesting initialization
                self._InitializeController()
                return
            if (lineParts[0] == "Active"):
                # controller requesting initialization
                self._IsActive()
                return
            if (lineParts[0] == 'p'):
                #self._printThis(lineParts)
                return
            if (lineParts[0] == 'w'):
                #self._sendspeeds(lineParts)
                return

    def _printThis(self, lineParts):
        '''
        Function to receive information from Arduino
        '''
        partsCount = len(lineParts)
        for x in range(1, partsCount):
            rospy.loginfo("Printing: " + lineParts[x])

    def _Pose(self, posereceived):
        '''
        Function to receive information from Arduino
        '''
        x = posereceived.pose.position.x
        y = posereceived.pose.position.y
        z = posereceived.pose.position.z
        message = 'G0 %0.4f %0.4f %0.4f\r\n' % (x,y,z)
        rospy.loginfo(message)
        return
 
    def _JointsPublisher(self, lineParts):
        '''
        Publish Joint state and end effector position
        '''
        partsCount = len(lineParts)
        #rospy.logwarn(partsCount)
        if (partsCount  < 6):
            pass
        try:
            theta_1 = float(lineParts[1])
            theta_2 = float(lineParts[2])
            theta_3 = float(lineParts[3])
            theta_4 = float(lineParts[4])
            x = float(lineParts[5])
            y = float(lineParts[6])
            z = float(lineParts[7])

            #quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
            joints = JointState()
            joints.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
            joints.position = [theta_1, theta_2, theta_3, theta_4]
            #quaternion = Quaternion()
            #quaternion.x = 0.0
            #quaternion.y = 0.0
            #quaternion.z = sin(theta/2.0)
            #quaternion.w = cos(theta/2.0)
            rosNow = rospy.Time.now()
            self._JointStatePublisher.publish(joints)
            posenow = PoseStamped()
            #posenow.header.seq = 0 
            posenow.header.stamp = rosNow
            posenow.header.frame_id = "global_base"
            posenow.pose.position.x = x
            posenow.pose.position.y = y
            posenow.pose.position.z = z
            self._PosePublisher.publish(posenow)

        except:
            rospy.logwarn("Unexpected error:" + str(sys.exc_info()[0]))        

    def _WriteSerial(self, message):
        self._SerialDataGateway.Write(message)

    def __init__(self, port="/dev/ttyACM0", baudrate=57600):
        '''
        Initializes the receiver class. 
        port: The serial port to listen to.
        baudrate: Baud rate for the serial communication
        '''
        self._Counter = 0
        rospy.init_node('teensy_ragnar')              
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baudRate = int(rospy.get_param("~baudRate", 57600))
        rospy.loginfo("Starting with serial port: " + \
            port + ", baud rate: " + str(baudRate))
        self._SerialDataGateway = SerialDataGateway(port, baudRate, \
            self._HandleReceivedLine)
        # Initialize publishers and subscribers 
        rospy.Subscriber("ragnar_ref_pose_test", PoseStamped, self._Pose, \
            queue_size=10)
        self._JointStatePublisher = rospy.Publisher("ragnar_joints_state", \
            JointState, queue_size=10)
        self._PosePublisher = rospy.Publisher("ragnar_pose", PoseStamped, \
            queue_size=10)
        self._InfoPublisher = rospy.Publisher("service_info", String, \
            queue_size=10)
        # Create services to change the mass inertia matrix 
        self._ApparentMassService = rospy.Service('set_mass_matrix', \
            threeby3Matrix, self._sendMassMatrix)
        self._ApparentDampService = rospy.Service('set_damping_matrix', \
            threeby3Matrix, self._sendBMatrix)
        self._ApparentStiffnessService = rospy.Service('set_stiffness_matrix', \
            threeby3Matrix, self._sendKMatrix)        

    def Start(self):
        rospy.logdebug("Starting")
        self._SerialDataGateway.Start()
 
    def Stop(self):
        rospy.logdebug("Stopping")
        self._SerialDataGateway.Stop()

    def _sendMassMatrix(self, request):
        '''
        Function to send the mass matrix on service request
        '''
        m11 = request.threeby3Matrix.data[0]
        m12 = request.threeby3Matrix.data[1]
        m13 = request.threeby3Matrix.data[2]
        m21 = request.threeby3Matrix.data[3]
        m22 = request.threeby3Matrix.data[4]
        m23 = request.threeby3Matrix.data[5]
        m31 = request.threeby3Matrix.data[6]
        m32 = request.threeby3Matrix.data[7]
        m33 = request.threeby3Matrix.data[8]


        message = 'Mm'
        print(message)
        publishs = String()
        publishs.data = message
        self._InfoPublisher.publish(publishs)
        rospy.loginfo("Received Mass Matrix")
        return threeby3MatrixResponse()

    def _sendBMatrix(self, request):
        '''
        Function to send the mass matrix on service request
        '''
        b11 = request.threeby3Matrix.data[0]
        b12 = request.threeby3Matrix.data[1]
        b13 = request.threeby3Matrix.data[2]
        b21 = request.threeby3Matrix.data[3]
        b22 = request.threeby3Matrix.data[4]
        b23 = request.threeby3Matrix.data[5]
        b31 = request.threeby3Matrix.data[6]
        b32 = request.threeby3Matrix.data[7]
        b33 = request.threeby3Matrix.data[8]
        rospy.loginfo("Received Damp Matrix")
        message = 'Bm'
        return threeby3MatrixResponse()

    def _sendKMatrix(self, request):
        '''
        Function to send the mass matrix on service request
        '''
        k11 = request.threeby3Matrix.data[0]
        k12 = request.threeby3Matrix.data[1]
        k13 = request.threeby3Matrix.data[2]
        k21 = request.threeby3Matrix.data[3]
        k22 = request.threeby3Matrix.data[4]
        k23 = request.threeby3Matrix.data[5]
        k31 = request.threeby3Matrix.data[6]
        k32 = request.threeby3Matrix.data[7]
        k33 = request.threeby3Matrix.data[8]
        rospy.loginfo("Received Stiffness Matrix")
        message = 'Km'
        return threeby3MatrixResponse()

                
    def _sendImu(self, request):
        """ Service to enable or disable imu for odometry"""

        rospy.set_param("~imuenable", request.imu_active)
        if request.imu_active:
            message = 'imu\r'
            rospy.loginfo("Imu enabled")
        else:
            message = 'noimu\r'
            rospy.loginfo("Imu disabled")

            self._WriteSerial(message)
            return #imuactiveResponse()

    def _sendang(self, request):
        rospy.set_param("~angular_correction", request.avaluescale)
        message = 'ascale %d %d\r' % self._GetBaseAndExponent(request.avaluescale)
        rospy.loginfo("Sending correction value: " + message)
        self._WriteSerial(message)
        return #angularscaleResponse()

    def _sendlin(self, request):
        rospy.set_param("~linear_correction", request.lvaluescale)
        message = 'lscale %d %d\r' % self._GetBaseAndExponent(request.lvaluescale)
        rospy.loginfo("Sending correction value: " + message)
        self._WriteSerial(message)
        return #linearscaleResponse()
   
    def _InitializeController(self):
        """ Writes an initializing string to start the Controller communication """
        rospy.loginfo("Initializing Controller")
        message = "activate\r\n"
        self._WriteSerial(message)
        
    def _IsActive(self):
        rospy.loginfo("Is Active")


if __name__ == '__main__':
    teensy = Teensy()
    try:
        teensy.Start()
        rospy.spin()

    except rospy.ROSInterruptException:
        teensy.Stop()
