#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue May 22 11:52:10 2018

@author: nvidia
"""
import yaml

class DC_Motor(object):
    def __init__(self, pca_obj):
        self.import_settings()
        self.pca = pca_obj
        self.current_pwm = 0


    def import_settings(self):
        path = "../config/settings.yaml"
        settings = yaml.safe_load(open(path))
        dc_motor_pwm = settings['dc_motor']
        self.channel = dc_motor_pwm ['channel']
        self.pwm_min = dc_motor_pwm['pwm_min']
        self.pwm_max = dc_motor_pwm['pwm_max']
        self.pwm_midpoint = dc_motor_pwm['pwm_midpoint']
        self.pwm_midrange_max = dc_motor_pwm['pwm_midrange_max']
        self.pwm_midrange_min = dc_motor_pwm['pwm_midrange_min']

    def set_current_pwm(self, new_pwm):
        self.current_pwm = new_pwm

    def get_current_pwm(self):
        return self.current_pwm

    def max_min_pwm(self, pwm_val):
        return max(self.pwm_min,min(pwm_val,self.pwm_max))

    def set_pwm(self, pwm_val):
        pwm = self.max_min_pwm(pwm_val)
        self.pca.set_pwm(self.channel, 0, pwm)
        self.set_current_pwm(pwm)
