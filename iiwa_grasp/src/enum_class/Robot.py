#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 09:14:37 2020

@author: taeke
"""

from enum import Enum
class RobotState(Enum):
    INITIALIZING = 0
    IDLE = 1
    PLANNING = 2
    MOVINGROBOT = 3
        
class RobotError(Enum):
    NONE = 0
    GOALPOSEOUTSIDEMOVEGROUP = 1
    CANNOTFINDPLAN = 2