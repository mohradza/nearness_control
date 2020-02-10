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
numReadings   = 360
threshold     =   9.0
preturn_thresh = 5.0
plotCarts     =   1

# ~~ Variables ~~
global lastHDepthScan
global lastHNearnessScan
global lastVDepthScan
global lastHNearnessScan
global a_0, a_1, a_2, b_1, b_2
global forward_speed, yaw_rate
lastHDepthScan = [ 0 for i in range( numReadings ) ]
lastHNearnessScan = [ 0 for i in range( numReadings ) ]
lastVDepthScan = [ 0 for i in range( numReadings ) ]
lastVNearnessScan = [ 0 for i in range( numReadings ) ]
lastWFReconScan = [ 0 for i in range( numReadings ) ]
lastSFScan = [ 0 for i in range( numReadings ) ]

car_state = ''
len_hist = 500





def h_depth_scan_cb( msg ):
    global lastHDepthScan
    """ Process the laser scan message """
    lastHDepthScan = msg.data  #lastDepthLaserScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

def h_nearness_cb( msg ):
    global lastHNearnessScan
    """ Process the nearness array """
    lastHNearnessScan = msg.data  #lastDepthImageScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

def v_depth_scan_cb( msg ):
    global lastVDepthScan
    """ Process the nearness array """
    lastVDepthScan = msg.data  #lastDepthImageScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

def v_nearness_cb( msg ):
    global lastVNearnessScan
    """ Process the nearness array """
    lastVNearnessScan = msg.data  #lastDepthImageScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

minAng = -pi
maxAng = pi
rospy.init_node( 'scan_plot' , anonymous = True )

# rospy.Subscriber( "wfi/horiz/image_scan" , Float32MultiArray , image_scan_cb )
rospy.Subscriber( "nearness_controller/horiz_depth_reformat" , Float32MultiArray , depth_scan_cb )
rospy.Subscriber( "nearness_controller/horiz_nearness" , Float32MultiArray , nearness_cb )
rospy.Subscriber( "nearness_controller/horiz_sf_nearness" , Float32MultiArray , SF_cb )
rospy.Subscriber( "octomap_point_cloud_centers/sca" , Float32MultiArray , WF_recon_cb )

try:
	while ( not rospy.is_shutdown() ):
                lastDepthScanNP   = np.asarray( lastDepthScan )
		lastNearnessScanNP   = np.asarray( lastNearnessScan )
		lastWFReconScanNP   = np.asarray( lastWFReconScan )
		lastSFScanNP   = np.asarray( lastSFScan )

		plt.clf() # Clear all figures

		plt.figure(1)
		plt.subplot(3,1,1)
		# Horizontal Depth Scan: Array Index vs. Depth [m]
		# plt.figure(num=1, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')
		plt.plot( lastDepthScanNP , 'b.' )
		plt.hold( False )
		plt.xlim( [ 0 , 360 ] )
		plt.ylim( [ 0 , 15] )
		plt.xlabel("Array Index")
		plt.ylabel("Depth [m]")
		plt.title("Horiz. Depth (Array Values)")

		plt.subplot(3,1,2)
		# Horizontal Depth Scan: X [m] vs Y [m]
		# plt.figure(num=2, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')
		plt.plot( lastNearnessScanNP , 'b.' )
		plt.hold( False )
		plt.xlim( [ 0 , 360 ] )
		plt.ylim( [ 0 , 2 ] )
		plt.xlabel("Array Index")
		plt.ylabel("Nearness [1/m]")
		plt.title("Horiz. Nearness (Array Values)")

		plt.subplot(3,1,3)
		# Horizontal Nearness Scan: X [1/m] vs Y [1/m]
		# plt.figure(num=4, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')
		plt.plot( lastWFReconScan , 'b.' )
		plt.hold( False )
		plt.xlim( [ 0 , 360 ] )
		plt.ylim( [ 0 , 3 ] )
		plt.xlabel("Array Index")
		plt.ylabel("Nearness [1/m]")
		plt.title("SF Horiz. Nearness (Array Values)")

		plt.pause( 0.001 )
	plt.show()
except KeyboardInterrupt:
	pass
