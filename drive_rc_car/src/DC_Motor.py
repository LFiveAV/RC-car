#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue May 22 11:52:10 2018

@author: nvidia
"""
CHANNEL = 1


class DC_Motor(object):
    def __init__(self, pca_obj, ch=CHANNEL):
        self.pca = pca_obj
        self.channel = ch
        self.pwm_min = 220
        self.pwm_max = 410
        self.pwm_midpoint = 315
        self.current_pwm = 0

    def set_current_pwm(self, new_pwm):
        self.current_pwm = new_pwm

    def get_current_pwm(self):
        return self.current_pwm

    def max_min_pwm(self, pwm_val):
        if pwm_val < self.pwm_min:
            return self.pwm_min
        elif pwm_val > self.pwm_max:
            return self.pwm_max
        else:
            return pwm_val

    def set_pwm(self, pwm_val):
        pwm = self.max_min_pwm(pwm_val)
        self.pca.set_pwm(self.channel, 0, pwm) # Comment the meaning of the input
        self.set_current_pwm(pwm)
