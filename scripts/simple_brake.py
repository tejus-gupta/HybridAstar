#! /usr/bin/python

import rospy
from prius_msgs.msg import Control
from sensor_msgs.msg import Joy

if __name__ == '__main__':
    rospy.init_node('simple_brake', anonymous=True)

    pub = rospy.Publisher('prius', Control, queue_size=1)
    rate = rospy.Rate(10)

    command = Control()
    command.throttle = 0
    command.brake = 1
    command.shift_gears = Control.NO_COMMAND
    command.steer = 0

    while not rospy.is_shutdown():
        pub.publish(command)
        rate.sleep()
    
    exit()