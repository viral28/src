#!/usr/bin/env python

import roslib; roslib.load_manifest('sawRobotIO1394QtConsole')
import rospy
import sys
import math
from std_msgs.msg import String
from std_msgs.msg import Float64
from sawROS.msg import vctDoubleVec

from collections import deque
from mlabwrap import mlab
import numpy as np


class Velcity():
    def __init__(self):
        # ros init
        rospy.init_node('talker')

        # ros publisher
        self.pub_vel_ = rospy.Publisher('/MTMR/joint_velocity/joint1', Float64)
        self.pub_vel_fil_ = rospy.Publisher('/MTMR/filtered/joint_velocity', Float64)
        self.pub_pot_j1_ = rospy.Publisher('/MTMR/joint_pot_position/joint1', Float64)
        self.pub_pot_fil_ = rospy.Publisher('/MTMR/filtered/joint_pot_position', Float64)
        self.pub_pot_vel_ = rospy.Publisher('/MTMR/pot_velocity/joint1', Float64)

        # ros subscriber
        self.sub_vel_ = rospy.Subscriber('/MTMR/joint_velocity',
                                         vctDoubleVec,
                                         self.vel_cb)
        self.sub_pot_ = rospy.Subscriber('/MTMR/pot_position',
                                         vctDoubleVec,
                                         self.pot_cb)
        self.sub_pot_vel_ = rospy.Subscriber('/MTMR/pot_velocity',
                                             vctDoubleVec,
                                             self.pot_vel_cb)
        
        # class variable
        self.joint_index_ = 0
        if len(sys.argv) == 2:
            arg1 = int(sys.argv[1])
            if arg1 >= 0 and arg1 <= 6:
                self.joint_index_ = arg1
                print "set joint_index to ", arg1
        
        self.vel_ = -1.0
        self.rate_ = rospy.Rate(50.0)
        sg_window_size = 51
        sg_order = 2
        sg_b, sg_g = mlab.sgolay(sg_order, sg_window_size, nout=2)
        coef_smooth = []
        coef_vel = []
        for i in range(sg_window_size):
            coef_smooth.append(sg_g[i][0])
            coef_vel.append(sg_g[i][1])
            
        self.sg_coef_ = np.array(coef_smooth)
        self.sg_coef_vel_ = np.array(coef_vel)

#        print self.sg_coef_
#        print self.sg_coef_vel_

        self.pot_history_ = deque(maxlen=sg_window_size)
        self.counter_ = 0;
        pass


    def run(self):
        while not rospy.is_shutdown():
#            self.pub_.publish(Float64(self.vel_))
            self.rate_.sleep()
            pass

    def vel_cb(self, data):
#        rospy.loginfo("%s: %f", rospy.get_time(), data.data[0])
        self.vel_ = data.data[self.joint_index_] * math.pi / 180.0
        pass

    def pot_cb(self, data):
        self.pot_history_.append(data.data[self.joint_index_])
        self.counter_ = self.counter_ + 1
        if len(self.pot_history_) ==  self.pot_history_.maxlen and self.counter_%5 == 0:
            # sg filter here
            pot_fil = np.dot(self.sg_coef_, np.array(self.pot_history_))
#            print "filtered pot position = %f" % pot_fil
            self.pub_pot_fil_.publish(Float64(pot_fil))
            self.pub_pot_j1_.publish(Float64(data.data[self.joint_index_]))
            
            vel_fil = np.dot(self.sg_coef_vel_, np.array(self.pot_history_))
            vel_fil = vel_fil / 0.001  # depends on dt
            self.pub_vel_fil_.publish(Float64(vel_fil))
            self.pub_vel_.publish(Float64(self.vel_))
            pass

    def pot_vel_cb(self, data):
        if self.counter_%5 == 0:
            pot_vel = data.data[self.joint_index_]
            self.pub_pot_vel_.publish(Float64(pot_vel))
        else:
            pass
                
if __name__ == '__main__':
    try:
        vel = Velcity()
        vel.run()
    except rospy.ROSInterruptException:
        pass


