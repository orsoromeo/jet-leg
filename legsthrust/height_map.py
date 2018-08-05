# -*- coding: utf-8 -*-
"""
Created on Sun Aug  5 18:50:25 2018

@author: romeoorsolino
"""

class HeightMap:
    def get_height(self, footPosWF_x, footPosWF_y):
        if footPosWF_x > -0.35:
            height = 0.2
        else:
            height = 0.0
            
        return height