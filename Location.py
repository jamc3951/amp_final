#!/usr/bin/python2
from enum import Enum

#Actions and observations go clockwise from north:
'''
        812   
        703
        654
'''

class Location(Enum):
    Current = 0
    Forward = 1
    FwdRight = 2
    Right = 3
    BackRight = 4
    Back = 5
    BackLeft = 6
    Left = 7
    FwdLeft = 8
    
