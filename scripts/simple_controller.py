#! /usr/bin/python

import rospy
import sys
sys.path.append('/home/tejus/catkin_ws/src/car_demo/')
from prius_msgs.msg import Control
from sensor_msgs.msg import Joy

if __name__ == '__main__':
    rospy.init_node('simple_controller', anonymous=True)
    
    pub = rospy.Publisher('prius', Control, queue_size=1)
    rate = rospy.Rate(10)
    
    command = Control()
    command.throttle = 1
    command.brake = 0
    command.shift_gears = Control.FORWARD
    command.steer = 0
    
    while not rospy.is_shutdown():
        pub.publish(command)
        rate.sleep()
    
    exit()