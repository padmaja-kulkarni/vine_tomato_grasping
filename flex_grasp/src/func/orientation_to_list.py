#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 10:09:14 2020

@author: taeke
"""

def orientation_to_list(orientation_msg):
    orientation = []

    orientation.append(orientation_msg.x)
    orientation.append(orientation_msg.y)
    orientation.append(orientation_msg.z)
    orientation.append(orientation_msg.w)

    return orientation
