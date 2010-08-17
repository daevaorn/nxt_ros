#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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


import roslib; roslib.load_manifest('nxt_controllers')  
import rospy
import math
import thread
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nxt_msgs.msg import Range, JointCommand


WHEEL_RADIUS = 0.044/2.0
WHEEL_BASIS  = 0.11/2.0
VEL_TO_EFF = 0.5
K_ROT = 0.075/VEL_TO_EFF
K_TRANS = 0.055/VEL_TO_EFF


class BaseController:
    def __init__(self):
        self.initialized = False
        self.vel_rot_desi = 0
        self.vel_trans_desi = 0
        self.vel_trans = 0
        self.vel_rot = 0


        # get joint name
        self.l_joint = rospy.get_param('l_wheel_joint', 'l_wheel_joint')
        self.r_joint = rospy.get_param('r_wheel_joint', 'r_wheel_joint')
        
        # joint interaction
        self.pub = rospy.Publisher('joint_command', JointCommand)
        rospy.Subscriber('joint_states', JointState, self.jnt_state_cb)

        # base commands
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        


    def cmd_vel_cb(self, msg):
        self.vel_rot_desi = msg.angular.z
        self.vel_trans_desi = msg.linear.x


    def jnt_state_cb(self, msg):
        velocity = {}
        for name, vel in zip(msg.name, msg.velocity):
            velocity[name] = vel

        # lowpass for measured velocity
        self.vel_trans = 0.5*self.vel_trans + 0.5*(velocity[self.r_joint] + velocity[self.l_joint])*WHEEL_RADIUS/2.0
        self.vel_rot =   0.5*self.vel_rot   + 0.5*(velocity[self.r_joint] - velocity[self.l_joint])*WHEEL_RADIUS/(2.0*WHEEL_BASIS)

        # velocity commands
        vel_trans = self.vel_trans_desi + K_TRANS*(self.vel_trans_desi - self.vel_trans)
        vel_rot = self.vel_rot_desi + K_ROT*(self.vel_rot_desi - self.vel_rot)
        
        # wheel commands
        l_cmd = JointCommand()
        l_cmd.name = self.l_joint
        l_cmd.effort = VEL_TO_EFF*(vel_trans/WHEEL_RADIUS - vel_rot*WHEEL_BASIS/WHEEL_RADIUS)
        self.pub.publish(l_cmd)

        r_cmd = JointCommand()
        r_cmd.name = self.r_joint
        r_cmd.effort = VEL_TO_EFF*(vel_trans/WHEEL_RADIUS + vel_rot*WHEEL_BASIS/WHEEL_RADIUS)
        self.pub.publish(r_cmd)


def main():
    rospy.init_node('base_controller')
    base_controller = BaseController()
    rospy.spin()



if __name__ == '__main__':
    main()
