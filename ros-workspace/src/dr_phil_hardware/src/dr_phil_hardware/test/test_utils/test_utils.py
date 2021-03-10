

import unittest
import math
import numpy as np



def assertRayEquals(test,r_correct,r_true,rel_tol=0,abs_tol=1e-8,msg=""):
    """ asserts that the two rays are approximately the same based on their distance as points"""
    test.assertTrue(math.isclose(np.linalg.norm(r_true.get_vec()-r_correct.get_vec()),0,abs_tol=abs_tol,rel_tol=rel_tol)
        ,"camera ray was off by {}. {}".format(np.linalg.norm(r_true.get_vec()-r_correct.get_vec()),msg))
