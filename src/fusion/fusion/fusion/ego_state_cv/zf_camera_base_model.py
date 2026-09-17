'''
 # Copyright 2024 ZF. All Rights Reserved.
 # Author:
 #        xuheng.chai@zf.com
 #####################################################################
 # @file
 # @brief Camera Base Parameters class.
'''

class CameraBaseModel():
    """ Camera Base Parameters class.
    """

    def __init__(self):
        self._param = None
        self._img_width = 1080
        self._img_height = 1920

    def set_parameters(self, param):
        self._param = param
        self._img_width = param.width
        self._img_height = param.height

    """ space2plane(point3D) projects a 3D point on to the image.
    If points are projected on the outside of the fov, return (-1,-1).
    -------Parameters
    point3D : numpy array or list([x, y, z]) array of points in camera coordinate (3xN)
    -------Returns
    point2D : numpy array array of points in image (2xN)
    """
    def space2plane(self, point3D):
        return self.world2cam(point3D)

    # Projects 3D points to the image plane (Pi function)
    """ liftProjective(point2D) lifts a point from the image plane to its projective ray.
    ----------Parameters
    point2D : numpy array or list([u,v])
        array of point in image 2xN
    ----------Returns
    point3D : numpy array
        array of point on unit sphere 3xN
    """
    def liftProjective(self, point2D):
        return self.cam2world(point2D)

    # Projects undistorted 2D points p_u to the image plane; f is focal length
    def undist2plane(self, f=1.0):
        pass

    # Projects 3D points to the image plane (Pi function)
    def world2cam(self, point3D):
        pass

    def cam2world(self, point2D):
        pass

    def calibrate(self, dir_name):
        pass