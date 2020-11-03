#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, time, numpy as np
import rospy, sensor_msgs.msg, geometry_msgs.msg

class Node:
    def __init__(self):
        #self.odrive = SerialOdrive()
        rospy.init_node('joystick_teleop')
        self.twist_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.joy_callback)

    def run(self, freq):
        while not rospy.is_shutdown():
            time.sleep(1)


    def joy_callback(self, msg):
        self.last_input = rospy.get_rostime()
        # yaw >0 left
        # pitch >0 front(down)
        _y, _p, _t, _r = msg.axes[:4]
        _t = -_t; _p=-_p
        #print('r{:.2f} p{:.2f} y{:.2f} t{:.2f}'.format(_r, _p, _y, _t))
        msg =  geometry_msgs.msg.Twist()
        msg.linear.x, msg.linear.y, msg.linear.z  = _p, _r, _t
        msg.angular.x, msg.angular.y, msg.angular.z = 0., 0., _y
        self.twist_pub.publish(msg)


def main(args):
    Node().run(20.)

if __name__ == '__main__':
    main(sys.argv)
