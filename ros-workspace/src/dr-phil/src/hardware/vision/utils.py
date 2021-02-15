#!/usr/bin/env python3

import numpy as np

def invert_homog_mat(hm):
    """ inverts homogenous matrix expressing rotation and translation in 3D or 2D """  
    
    # if 3D transformation matrix
    if hm.shape[0] == 4 == hm.shape[1]:
        newH = 3
    elif hm.shape[0] == 3 == hm.shape[1]:
        newH = 2
    else:
        raise Exception("Invalid matrix shape: " + hm.shape)
        
    rotI = hm[0:newH,0:newH].T
    transI = -rotI @ hm[0:newH,newH]

    hm[0:newH,0:newH] = rotI
    hm[0:newH,newH] = transI

    return hm

