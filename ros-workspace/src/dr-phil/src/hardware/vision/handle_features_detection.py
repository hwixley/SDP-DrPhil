import numpy as np


def get_handle_features(handle_type,cropped_image):
    """ given handle type (0 = vertical twisting handle, 1 = pull/push handle, 2 = bar handle) and cropped image in line with heuristic
        detects (p1,p2,p3) important points on the handle:
            for:
                0 - p1 is the point in line with the keyhole, and p2 the tip of the handle
                1 - p1 is the middle of the handle 
                2 - p1 is the left axis, p2 is the right axis, and p3 is the middle of the push bar     
    """
    p1 = np.array([0,0])
    p2 = np.array([0,0])
    p3 = np.array([0,0])

    return (p1,p2,p3)