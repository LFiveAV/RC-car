#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue May 22 11:52:10 2018

@author: nvidia
"""
import yaml

CHANNEL = 0


class ServoMotor(object):
    def __init__(self, pca_obj):
        self.import_settings()
        self.pca = pca_obj
        self.current_pwm = 0

    def import_settings(self):
        path = "../config/settings.yaml"
        settings = yaml.safe_load(open(path))
        servo_pwm = settings['servo']
        self.channel = servo_pwm['channel']
        self.pwm_min = servo_pwm['pwm_min']
        self.pwm_max = servo_pwm['pwm_max']
        self.pwm_midpoint = servo_pwm['pwm_midpoint']

    def set_current_pwm(self, new_pwm):
        self.current_pwm = new_pwm

    def get_current_pwm(self):
        print("get_current_pwm")
        return self.current_pwm

    def max_min_pwm(self, pwm_val):
        return max(self.pwm_min,min(pwm_val,self.pwm_max))

    def set_pwm(self, pwm_val):
        pwm = self.max_min_pwm(pwm_val)
        self.pca.set_pwm(self.channel, 0, pwm)     # send new pwm value over i2c
        self.set_current_pwm(pwm)
