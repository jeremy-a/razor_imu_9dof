#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# new node for 9DoF Razor IMU M0
# https://www.sparkfun.com/products/14001

import rospy
import serial
import string
import math
import sys

#from time import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
# from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# Callback for dynamic reconfigure requests
#TODO make reconfig to setup IMU for a, g, and q (and maybe e for diag)
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

rospy.init_node("razor_node2")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('imu', Imu, queue_size=1)
#srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

imuMsg = Imu()

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

default_port='/dev/ttyACM0'

#read calibration parameters
port = rospy.get_param('~port', default_port)

# calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
# magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
# magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
# imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# Check your COM port and baud rate
baudrate=115200
wait_for_serial = True
ser = None

while wait_for_serial:
    rospy.loginfo("Opening %s @ %d...", port, baudrate)
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        wait_for_serial = False
        rospy.loginfo("IMU found at port " + port + " and baud " + str(baudrate) + ".")
    except serial.serialutil.SerialException:
        rospy.logerr("IMU not found at port " + port + " and baud " + str(baudrate) + ". Did you specify the correct port in the launch file? retrying ...")
        #sleep and retry
        rospy.sleep(1)

# roll=0
# pitch=0
# yaw=0
# seq=0
accel_factor = 9.806 / 2.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
# rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
# rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

#at this point assume connection but not necessarily data.

#check setup run state
# ser.write(' ')
# oneline = ser.readline() #or just check empty?
# words = string.split(line, ",")  # Fields split
# if not words[0].isdigit():
#     ser.write(' ') # toggle it
# # todo if bad here then exception/error/quit?

#see if eulers are on

seq=0

while not rospy.is_shutdown():
    line = ser.readline()
    words = string.split(line,",")    # Fields split
    rospy.logdebug("len %f first %d second %f", len(words), int(words[0]), float(words[1]))
    if len(words) == 11:

        # Publish message
        #REP 103
        #angular
        # roll rotation about X
        # pitch rotation about Y
        # yaw rotation about Z
        #body
        # X forward
        # Y left
        # Z up
        imuMsg.linear_acceleration.x = float(words[1]) * accel_factor
        imuMsg.linear_acceleration.y = float(words[2]) * accel_factor
        imuMsg.linear_acceleration.z = float(words[3]) * accel_factor

        imuMsg.angular_velocity.x = float(words[4])
        imuMsg.angular_velocity.y = float(words[5])
        imuMsg.angular_velocity.z = float(words[6])

        imuMsg.orientation.w = float(words[7])
        imuMsg.orientation.x = float(words[8])
        imuMsg.orientation.y = float(words[9])
        imuMsg.orientation.z = float(words[10])
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'base_imu_link'
        imuMsg.header.seq = seq
        seq = seq + 1
        pub.publish(imuMsg)

    # if (diag_pub_time < rospy.get_time()) :
    #     diag_pub_time += 1
    #     diag_arr = DiagnosticArray()
    #     diag_arr.header.stamp = rospy.get_rostime()
    #     diag_arr.header.frame_id = '1'
    #     diag_msg = DiagnosticStatus()
    #     diag_msg.name = 'Razor_Imu'
    #     diag_msg.level = DiagnosticStatus.OK
    #     diag_msg.message = 'Received AHRS measurement'
    #     diag_msg.values.append(KeyValue('roll (deg)',
    #                             str(math.degrees(roll))))
    #     diag_msg.values.append(KeyValue('pitch (deg)',
    #                             str(math.degrees(pitch))))
    #     diag_msg.values.append(KeyValue('yaw (deg)',
    #                             str(math.degrees(yaw))))
    #     diag_msg.values.append(KeyValue('sequence number', str(seq)))
    #     diag_arr.status.append(diag_msg)
    #     diag_pub.publish(diag_arr)

ser.close
#f.close
