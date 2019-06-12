# -*- coding: utf-8 -*-
"""
Created on Thu Jan 12 17:41:10 2017

@author: cunnia3
"""

class PID:
    """
    discrete PID control for finding next command u[k]
    u[k] = au[k-1] + be[k] + ce[k-1] + de[k-2]
    """
    def __init__(self, a=0.0, b=0.0, c=0.0, d=0.0):
        self.sat = 6000

        self.a = a
        self.b = b
        self.c = c
        self.d = d
         
        self.set_point = 0     # reference
        self.e = 0             # current error
        self.pe = 0            # previous error
        self.ppe = 0           # error at e[k-2]
        self.u = 0             # current command
        self.pu = 0            # previous command
 
    def update(self, measurement, feedforward = 0):
        """ Use measurement to come up with new command """
        if self.set_point == 0:
            return 0

        self.pu = self.u
        self.ppe = self.pe
        self.pe = self.e
        
        self.e = self.set_point - measurement         
        self.u = feedforward + self.a * self.pu + self.b*self.e + self.c*self.pe + self.d*self.ppe
        if self.u > self.sat:
            self.u = self.sat
        if self.u < -self.sat:
            self.u = -self.sat
        
        return self.u
        
    def setPoint(self, set_point):
        self.set_point = set_point

        if set_point == 0:
            self.e=0
            self.u=0
            self.pe=0
            self.pu = 0
            self.ppe = 0
        
    def getPoint(self):
        return self.set_point
        
    def getError(self):
        return self.e
