#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from asyncore import write
import rospy
from sensor_msgs.msg import JointState
from math import pi
import serial
import serial.tools.list_ports as listport

def callback(data):
    shoulder_base_angle = int(round(radian_to_degree(data.position[0])))
    shoulder_arm_angle = int(round(radian_to_degree(data.position[1])))
    elbow_angle = int(round(radian_to_degree(data.position[2])))
    wrist_pitch_angle = int(round(radian_to_degree(data.position[3])))
    wrist_roll_angle = int(round(radian_to_degree(data.position[4])))

    serial_message = serial_message_generator(shoulder_base_angle, shoulder_arm_angle, elbow_angle, wrist_pitch_angle, wrist_roll_angle)

    print("\n### Node Started: joint_position_reader ###\n")
    print("serial message: ",serial_message)
    print("shoulder_base_angle: ",shoulder_base_angle)
    print("shoulder_arm_angle: ", shoulder_arm_angle)
    print("elbow_angle", elbow_angle)
    print("wrist_pitch_angle", wrist_pitch_angle)
    print("wrist_roll_angle", wrist_roll_angle)
    print("-" * 60)

    try:
        port = serial.Serial(port = '/dev/ttyUSB0',baudrate=9600)
        ports = listport.comports()
        port.write(bytes(serial_message,'utf-8'))
    except:
        print("Serial Connection Error")
        print("No data is sended!")


def radian_to_degree(radian):
    degree = (radian/pi)*180
    return degree


def listener():
    rospy.init_node("joint_position_reader")
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.spin()

def serial_message_generator(sh_base, sh_arm, elbow, wrist_p, wrist_r):
    message = ""
    frames = [sh_base,sh_arm,elbow,wrist_p,wrist_r]
    for frame in frames:
        if frame < 0:
            m_frame = "0"+str(abs(frame)).rjust(3,"0")
        else:
            m_frame = "1"+str(frame).rjust(3,"0")
        if frame == frames[-1]:
            message += m_frame
        else:
            message += m_frame
            message += "-"
    message = "S"+message+"CF"
    return message




if __name__ == '__main__':
    try:
        print("\n### Node Started: servo_joy ###\n")
        listener()
        rospy.sleep(0.08)
    except rospy.ROSInterruptException:
        pass
    
    
