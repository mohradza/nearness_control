#!/usr/bin/env python

import numpy as np
from numpy import pi
from math import sin, cos
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import pandas as pd

# ~~ Setting and Constants ~~
numReadings   = 920
numHorizScans = 460
threshold     =   9.0
preturn_thresh = 5.0
plotCarts     =   1

# ~~ Variables ~~
global lastDepthScan
global lastNearnessScan
global rawScan
global a_0, a_1, a_2, b_1, b_2
global forward_speed, yaw_rate
lastRawScan = [ 0 for i in range( numReadings ) ]
lastDepthScan = [ 0 for i in range( numHorizScans ) ]
lastNearnessScan = [ 0 for i in range( numHorizScans ) ]
lastWFReconScan = [ 0 for i in range( numHorizScans ) ]
lastSFScan = [ 0 for i in range( numHorizScans ) ]

car_state = ''
len_hist = 500

def raw_scan_cb( msg ):
    global lastRawScan
    lastRawScan = msg.ranges

def depth_scan_cb( msg ):
    global lastDepthScan
    """ Process the laser scan message """
    lastDepthScan = msg.data  #lastDepthLaserScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

def nearness_cb( msg ):
    global lastNearnessScan
    """ Process the nearness array """
    lastNearnessScan = msg.data  #lastDepthImageScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

def WF_recon_cb( msg ):
    global lastWFReconScan
    """ Process the nearness array """
    lastWFReconScan = msg.data  #lastDepthImageScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

def SF_cb( msg ):
    global lastSFScan
    """ Process the nearness array """
    lastSFScan = msg.data  #lastDepthImageScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

minAng = -pi
maxAng = pi
rospy.init_node( 'scan_plot' , anonymous = True )

# rospy.Subscriber( "wfi/horiz/image_scan" , Float32MultiArray , image_scan_cb )
rospy.Subscriber( "/MARBLE_HUSKY/planar_laser_scan" , LaserScan , raw_scan_cb )
rospy.Subscriber( "/MARBLE_HUSKY/nearness_controller/horiz_depth_reformat" , Float32MultiArray , depth_scan_cb )
rospy.Subscriber( "/MARBLE_HUSKY/nearness_controller/horiz_nearness" , Float32MultiArray , nearness_cb )
rospy.Subscriber( "/MARBLE_HUSKY/nearness_controller/horiz_sf_nearness" , Float32MultiArray , SF_cb )
rospy.Subscriber( "/MARBLE_HUSKY/nearness_controller/horiz_recon_wf_nearness" , Float32MultiArray , WF_recon_cb )

try:
	while ( not rospy.is_shutdown() ):
                lastDepthScanNP   = np.asarray( lastDepthScan )
		lastNearnessScanNP   = np.asarray( lastNearnessScan )
		lastWFReconScanNP   = np.asarray( lastWFReconScan )
		lastSFScanNP   = np.asarray( lastSFScan )

		plt.clf() # Clear all figures

		plt.figure(1)
		plt.subplot(4,1,1)
		# Horizontal Depth Scan: Array Index vs. Depth [m]
		# plt.figure(num=1, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')
		plt.plot( lastRawScan , 'b.' )
		#plt.hold( False )
		plt.xlim( [ 0 ,  920] )
		plt.ylim( [ 0 , 25 ] )
		plt.xlabel("Array Index")
		plt.ylabel("Depth [m]")
		plt.title("Horiz. Depth (Array Values)")
		plt.subplot(4,1,2)
		# Horizontal Depth Scan: Array Index vs. Depth [m]
		# plt.figure(num=1, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')
		plt.plot( lastDepthScanNP , 'b.' )
		#plt.hold( False )
		plt.xlim( [ 0 ,  460] )
		plt.ylim( [ 0 , 25 ] )
		plt.xlabel("Array Index")
		plt.ylabel("Depth [m]")
		plt.title("Horiz. Depth (Array Values)")

		plt.subplot(4,1,3)
		# Horizontal Depth Scan: X [m] vs Y [m]
		# plt.figure(num=2, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')
		plt.plot( lastNearnessScanNP , 'b.' )
		plt.hold( True )
                plt.plot(lastWFReconScanNP , 'r.' )
		plt.hold( False )

		plt.xlim( [ 0 , 460 ] )
		plt.ylim( [ 0 , 1 ] )
		plt.xlabel("Array Index")
		plt.ylabel("Nearness [1/m]")
		plt.title("Horiz. Nearness (Array Values)")

		plt.subplot(4,1,4)
		# Horizontal Nearness Scan: X [1/m] vs Y [1/m]
		# plt.figure(num=4, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')
		plt.plot( lastSFScanNP , 'b.' )
		#plt.hold( False )
		plt.xlim( [ 0 , 390] )
		plt.ylim( [ 0 , 1.0 ] )
		plt.xlabel("Array Index")
		plt.ylabel("Nearness [1/m]")
		plt.title("SF Horiz. Nearness (Array Values)")

		plt.pause( 0.001 )
	plt.show()
except KeyboardInterrupt:
	pass
