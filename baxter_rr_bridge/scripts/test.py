#!/usr/bin/env python

import rospy
import baxter_interface
import time

rospy.init_node('Hello_Baxter')
limb = baxter_interface.Limb('right')
angles = limb.joint_angles()
print angles
angles['right_s0']=-0.1
angles['right_s1']=-0.1
angles['right_e0']=-0.1
angles['right_e1']=-0.1
angles['right_w0']=-0.1
angles['right_w1']=-0.1
angles['right_w2']=-0.1

s = limb.joint_angles()
print (s['right_w2']-angles['right_w2'])
print 'start:'
start_time = time.time()

limb.move_to_joint_positions(s)
elapsed_time1 = time.time() - start_time
start_time1 = time.time()

limb.move_to_joint_positions(angles)
elapsed_time2 = time.time() - start_time1
start_time2 = time.time()
s = limb.joint_angles()
elapsed_time3 = time.time() - start_time2

print elapsed_time1
print elapsed_time2
print elapsed_time3
#while ((s['right_w2']-angles['right_w2'])>0.05):
#    elapsed_time2 = time.time() - start_time
#    s = limb.joint_angles()    
print 'end'

quit()
