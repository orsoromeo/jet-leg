# -*- coding: utf-8 -*-
"""
Created on Sat Nov 24 15:20:49 2018

@author: Romeo Orsolino
"""
import numpy as np
from math_tools import Math
from grid_map_msgs.msg import  GridMap

class gridMapConverter:
    def __init__(self):
        self.gridmap = 0
        
    def getParamsFromRosDebugTopic(self, received_data):
        print 'message layers', received_data.layers
        print 'message basic layers', received_data.basic_layers
        print 'message lenght x', received_data.info.length_x
        for currentData in received_data.data:
            print 'message dataaaaaaaaa!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            size_x = currentData.layout.dim[0].size
            size_y = currentData.layout.dim[1].size
            print 'size x ', size_x
            print 'size y ', size_y