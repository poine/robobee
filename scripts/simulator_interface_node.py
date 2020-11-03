#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time, numpy as np
import rospy

import pat3.ros_utils as p3_rpu
import common_control.rospy_utils as cc_rpu

class Node(p3_rpu.PeriodicNode):

    def __init__(self):
        p3_rpu.PeriodicNode.__init__(self, 'simulator_coms_node')
        self.transform_pub = p3_rpu.TransformPublisher()
        self.pose_lst = cc_rpu.PoseListener('/drone/gt_pose')
        
    def periodic(self):
        now = rospy.Time.now()
        self.transform_pub.send_w_enu_to_ned_transform(now)
        try:
            t, q = self.pose_lst.get_t_and_q()
            T_w2b = p3_rpu.T_of_t_q(t, q)
            self.transform_pub.send_w_enu_to_b_transform(now, T_w2b, 'b_flu')
        except cc_rpu.RobotNotLocalizedException:
            pass

def main(args):
    Node().run(20.)

if __name__ == '__main__':
    main(sys.argv)
