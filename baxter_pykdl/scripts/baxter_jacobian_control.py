#!/usr/bin/python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
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

import rospy

from baxter_pykdl import baxter_kinematics
import baxter_interface
import time
import numpy as np


def main():
    rospy.init_node('baxter_kinematics')
    print '*** Baxter PyKDL Kinematics ***\n'
    kin = baxter_kinematics('right')

    # build jacobian
    jacobian=kin.jacobian()
    
    # create limb interface
    limb = baxter_interface.Limb('right')
    angles = limb.joint_angles()
    angles['right_s0']=0.0
    angles['right_s1']=0.0
    angles['right_e0']=0.0
    angles['right_e1']=0.0
    angles['right_w0']=0.0
    angles['right_w1']=0.0
    angles['right_w2']=0.0
    
    #angles are initialized
    start= time.time()
    # run for 2 seconds
    while time.time()-start < 2:
        jacobianInv=kin.jacobian_pseudo_inverse()
        # includes angular and linear velocities
        # desriedEndEffectorVelocities = np.array([Vx,Vy,Vz,Wx,Wy,Wz])
        desiredEndEffectorVelocities = np.array([[-.1],[0],[0],[0],[0],[0]])
        
        # q' = J(q)^-1*[Vx;Wx]
        jointVelocities = jacobianInv*desiredEndEffectorVelocities

        angles['right_s0']=jointVelocities[0]
        angles['right_s1']=jointVelocities[1]
        angles['right_e0']=jointVelocities[2]
        angles['right_e1']=jointVelocities[3]
        angles['right_w0']=jointVelocities[4]
        angles['right_w1']=jointVelocities[5]
        angles['right_w2']=jointVelocities[6]



        #print jointVelocities
        print jointVelocities
        limb.set_joint_velocities(angles)
        time.sleep(.1)



if __name__ == "__main__":
    main()
