#!/usr/bin/env python
# license removed for brevity

import rospy
import pygame
from pygame import joystick
from geometry_msgs.msg import TwistStamped

pygame.init()
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]

#Speed Coefficients Decleration
linear_speed_coeff = 3
angular_speed_coeff = 1


for joystick in joysticks:
    print(joystick.get_name())


def send_commands():

    global linear_speed_coeff 
    global angular_speed_coeff 

    print("\n### Node Started: servo_joy ###\n")

    pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
    rospy.init_node('servo_joy', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    # Twist command
    command = TwistStamped()

    while not rospy.is_shutdown():
        # Joystick event check
        for event in pygame.event.get():
            print(event)
            if event.type == 1536: # JOYAXISMOTION description
                print(event)
                if event.axis == 1: # vertical axis
                    command.twist.linear.x = -event.value*linear_speed_coeff
                if event.axis == 0: # horizontal axis
                    command.twist.linear.y = -event.value*linear_speed_coeff
                if event.axis == 4: # horizontal axis
                    command.twist.linear.z = -event.value*linear_speed_coeff
                if event.axis == 3: # horizontal axis
                    command.twist.angular.x = -event.value*angular_speed_coeff
                if event.axis == 4: # horizontal axis
                    command.twist.angular.y = -event.value*angular_speed_coeff
            if event.type == 1538:
                if event.value == (-1,0):
                    command.twist.angular.z = -angular_speed_coeff
                if event.value == (1,0):
                    command.twist.angular.z = angular_speed_coeff
            
            if event.type == 772:
                command.twist.linear.x = 0
                command.twist.linear.y = 0
                command.twist.linear.z = 0
                command.twist.angular.x = 0
                command.twist.angular.y = 0
                command.twist.angular.z = 0
                
        pub.publish(command)
        rate.sleep()
        #print(command)

if __name__ == '__main__':
    try:
        send_commands()
    except rospy.ROSInterruptException:
        pass
