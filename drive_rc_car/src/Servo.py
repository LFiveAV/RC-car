#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue May 22 11:52:10 2018

@author: nvidia
"""


CHANNEL = 0


class ServoMotor(object):
    def __init__(self, pca_obj, ch=CHANNEL):
        self.pca = pca_obj
        self.channel = ch
        self.pwm_min = 205
        self.pwm_max = 410
        self.pwm_midpoint = 308
        self.current_pwm = 0

    def set_current_pwm(self, new_pwm):
        self.current_pwm = new_pwm

    def get_current_pwm(self):
        print("get_current_pwm")
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
        self.pca.set_pwm(self.channel, 0, pwm)     # send new pwm value over i2c
        self.set_current_pwm(pwm)