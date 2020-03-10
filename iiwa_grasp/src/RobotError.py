#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 08:58:39 2020

@author: taeke
"""

from enum import Enum
class RobotError(Enum):
    NONE = 0
    GOALPOSEOUTSIDEMOVEGROUP = 1
    CANNOTFINDPLAN = 2