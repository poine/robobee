#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, time, numpy as np
import rospy, geometry_msgs.msg

import common_control.rospy_utils as cc_rpu
import pat3.ros_utils as p3_rpu

import math
def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

class HoverController:
    def __init__(self):
        self.wp_sp = np.array([2., 1.5, 1.5])
        self.psi_sp = np.deg2rad(0)#-45)#0.

    def get(self, pose, ori):
        fmt = 'pose {:.2} {:.2f} {:.2f}, {:.1f} deg -> sp {:.2} {:.2f} {:.2f}, {:.1f} deg'
        print(fmt.format(pose[0], pose[1], pose[2], np.rad2deg(ori[2]), self.wp_sp[0], self.wp_sp[1], self.wp_sp[2], np.rad2deg(self.psi_sp)))
        pos_err_w = pose - self.wp_sp
        psi = ori[2]; cpsi, spsi = np.cos(psi), np.sin(psi)
        # vclp: vehicle carried local plan
        R_w2vclp = np.array([[cpsi, spsi, 0],[-spsi, cpsi, 0], [0, 0, 1.]])
        pos_err_vclp = np.dot(R_w2vclp, pos_err_w)
        vel_sp_vclp = np.multiply([-0.5, -0.5, -0.5], pos_err_vclp)             # gain
        vel_sp_vclp = np.clip(vel_sp_vclp, [-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]) # saturation
        psi_err = psi - self.psi_sp
        ax, ay, az = 0., 0., -0.5 * psi_err
        return vel_sp_vclp, (ax, ay, az)

class WPSeq:
    def __init__(self):
        self.seq = np.array([[0, 0, 1.5], [1, 0, 1.5], [1, 1, 1.5], [0, 1, 1.5]])
        self.idx = 0
    
class WPSeqCtl:
    def __init__(self, h_ctl):
        self.h_ctl = h_ctl

    def get(self, t, pose, ori):
        if 0:
            self.h_ctl.wp_sp[0] = step(t, a0=1., a1=4., dt=17, t0=0)
            self.h_ctl.wp_sp[1] = step(t, a0=1., a1=4., dt=20, t0=0)
            self.h_ctl.wp_sp[2] = step(t, a0=1.2, a1=1.4, dt=10, t0=0)
        else:
            om, om2 = 0.5*t, 0.33*t
            self.h_ctl.wp_sp[0] = 2. + 2*np.sin(om)
            self.h_ctl.wp_sp[1] = 2. + 2*np.cos(om)
            self.h_ctl.wp_sp[2] = 1.5+0.1*np.sin(om2)
        return self.h_ctl.get(pose, ori)

    
class Node(cc_rpu.PeriodicNode):
    def __init__(self):
        self.h_ctl = HoverController()
        self.wps_ctl = WPSeqCtl(self.h_ctl)
        cc_rpu.PeriodicNode.__init__(self, 'test_sjtu_drone')
        self.twist_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.transform_pub = p3_rpu.TransformPublisher()
        self.marker_pub = p3_rpu.PoseArrayPublisher(dae='quad_murat.dae')
        self.odom_pub = p3_rpu.OdomPublisher()
        self.pose_lst = cc_rpu.PoseListener('/drone/gt_pose')
 
    def periodic(self):
        now = rospy.Time.now()
        try: # FIXME!!!!
            t_enu, e_enu = self.pose_lst.get_loc_and_rpy()
            #T_w2b = p3_rpu.T_of_t_rpy(t_enu, e_enu)
            #self.transform_pub.send_w_enu_to_ned_transform(now)
            #self.transform_pub.send_w_enu_to_b_transform(now, T_w2b)
            #self.transform_pub.publish(now, T_w2b)
            t_ned = [t_enu[1], t_enu[0], -t_enu[2]]
            e_ned = [e_enu[0], -e_enu[1], e_enu[2]+np.pi/2] 
            T_b2wned = p3_rpu.T_of_t_rpy(t_ned, e_ned)
            #T_b2w = np.linalg.inv(T_w2b)
            self.marker_pub.publish([T_b2wned])
            self.odom_pub.publish(T_b2wned, now)
            cmd = self.wps_ctl.get(now.to_sec(), t_enu, e_enu)
            self.publish_twist(cmd)
        except cc_rpu.RobotNotLocalizedException:
            pass
    
    def publish_twist(self, cmd):
        print(cmd)
        msg =  geometry_msgs.msg.Twist()
        msg.linear.x, msg.linear.y, msg.linear.z  = cmd[0]
        msg.angular.x, msg.angular.y, msg.angular.z = cmd[1]
        #pdb.set_trace()
        self.twist_pub.publish(msg)
    
    
def main(args):
    Node().run(10.)

if __name__ == '__main__':
    main(sys.argv)
