'''
 # Copyright 2024 ZF. All Rights Reserved.
 # Author:
 #        xuheng.chai@zf.com
 #####################################################################
 # @file
 # @brief Camera Scaramuzza model class.
'''

import numpy as np
import cv2
from .zf_camera_base_model import CameraBaseModel
from .zf_camera_wide_parameters import CameraWideParameters


class CameraWideModel(CameraBaseModel):
    """ OCamCalib[1] unndistortion class.
    ----------Parameters
    fov : float
        field of view of the camera in degree
    ----------References
    [1] https://sites.google.com/site/scarabotix/ocamcalib-toolbox
    """

    def __init__(self, fov=360):
        self._fov = fov
        self._camera_matrix = np.identity(3)
        self.__new_camera_matrix = np.identity(3)
        self._dist_coeffs = np.zeros((4, 1))
        self._rotation = np.eye(3)
        self._translation = np.zeros((3, 1))

    def set_fov(self, fov):
        # field of view
        self._fov = fov

    def frame_camera2pixel(self, point3D_in_camera):
        return self.__world2cam(point3D_in_camera)

    """ frame_pixel2camera(point2D) projects a 2D point in origin image onto the unit sphere.
        param:
            point2D:    numpy array or list([x, y])
            Type:   array of points in origin image (2xN)

        return:
            point3D:    numpy array 
            Type:   array of unit points in camera coordinate (3xN)
    """
    def frame_pixel2camera(self, point2D):
        return self.__cam2world(point2D)

    def set_parameters(self, param: CameraWideParameters):
        if (not param.is_param_valid()):
            raise Exception("invalid parameters, please read the model config file first!")
        self._param = param
        self._img_width = param.width
        self._img_height = param.height
        self._camera_matrix = param._camera_matrix
        self._dist_coeffs = param._dist_coeffs
        imageSize = (int(param.height), int(param.width))
        new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K=self._camera_matrix,
            D=self._dist_coeffs,
            image_size=imageSize[::-1],
            R=None,
            P=None,
            balance=1.0,
            new_size=imageSize[::-1],
            # fov_scale=1.0
        )
        new_camera_matrix = self._camera_matrix.copy()
        self.undistortScale = 2.0
        new_camera_matrix[0, 0] = (self._camera_matrix[0, 0]) / 3.14 * self.undistortScale
        new_camera_matrix[1, 1] = (self._camera_matrix[1, 1]) / 3.14 * self.undistortScale
        # self.undistortOffset = (0, 0)
        # # if undist_offset is not None:
        # #     self.undistortOffset = undist_offset
        # #     new_camera_matrix[0, 2] += self.undistortOffset[1]
        # #     new_camera_matrix[1, 2] += self.undistortOffset[0]
        # # else:
        # #     self.undistortOffset = (0, 0)
        #
        # # new_camera_matrix = np.array([[undist_size[1] * undist_scale, 0.0, undist_offset[1]],
        # #                               [0.0, undist_size[0] * undist_scale, undist_offset[0]],
        # #                               [0.0, 0.0, 1.0]])

        self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(
            K=self._camera_matrix,
            D=self._dist_coeffs,
            R=None,
            P=new_camera_matrix,
            size=imageSize[::-1],
            m1type=cv2.CV_32FC1
        )

    """ frame_world2pixel(point3D, R, t) projects a 3D point in world frame to pixel frame.
        param:
            point3D: numpy array or list([x, y, z])
                array of points in world coordinate (3xN)
            R: numpy array
                rotation matrix 3x3
            t: numpy array
                translation matrix 3x1
        
        return:
            point2D : numpy array
                array of points in image (2xN)
    """
    def frame_world2pixel(self, point3D, R, t):
        point3D_in_camera = R @ point3D + t
        print(point3D_in_camera, np.linalg.norm(point3D_in_camera))
        return self.__world2cam(point3D_in_camera)

    """ frame_pixel2world(point3D, R, t) projects a 3D point in world frame to pixel frame.
        param:
            point3D: numpy array or list([x, y, z])
                array of points in world coordinate (3xN)
            R: numpy array
                rotation matrix 3x3
            t: numpy array
                translation matrix 3x1

        return:
            point2D : numpy array
                array of points in image (2xN)
    """
    def frame_pixel2world(self, point2D, R, t, norm2=1):
        unit_point3D = self.frame_undistort_pixel2sphere(point2D)
        Rinv = np.transpose(R)
        tinv = -1 * Rinv @ t
        point3D_in_world = R.dot(unit_point3D * norm2)
        return point3D_in_world

    """ frame_undistort_pixel2sphere(point2D) projects a 2D point in undistort image onto the unit sphere.
        param:
            point2D : numpy array or list([x, y])
                array of points in undistort image (2xN)

        return:
            point3D: numpy array 
                array of points in world coordinate (3xN)
    """
    def frame_undistort_pixel2sphere(self, point2D):
        point2D_ori = self.frame_undistort_pixel2origin(point2D)
        return self.__cam2world(point2D_ori)

    def frame_undistort_pixel2origin(self, point2D, f=1.0):
        H = self._img_height
        W = self._img_width
        focal = abs(f)
        focal = 0.2 if focal < 0.2 else focal
        z = W / focal
        x = point2D[0] - W / 2
        y = point2D[1] - H / 2
        point3D = np.array([x, y, z]).reshape(3, -1)
        point2D_ori = self.__world2cam(point3D)
        print("point2D_ori: ",  point2D_ori)
        return point2D_ori


    # Perspective projection
    def undist2plane_perspective(self):
        if (not self._param):
            raise Exception("no parameters, please read the model config file first!")
        if (not self._param.is_param_valid()):
            raise Exception("invalid parameters, please read the model config file first!")
        return self.mapx, self.mapy

    def __cam2world(self, point2D):
        """ __cam2world(point2D) projects a 2D point onto the unit sphere.
        In this function fov of the camera is not considered.
        The coordinate is different than that of the original OcamCalib.
        point3D coord: x:right direction, y:down direction, z:front direction
        point2D coord: x:row direction, y:col direction (OpenCV image coordinate)

        Parameters
        ----------
        point2D : numpy array or list([u,v])
            array of point in image 2xN

        Returns
        -------
        point3D : numpy array
            array of point on unit sphere 3xN

        Examples
        --------
        ocam.__cam2world([502,900]).tolist() # project a point onto unit sphere
        [[-0.5776824148317081], [0.20599312860435134], [0.7898416667674589]]
        tmp = ocam.__cam2world(1600*np.random.rand(2, 10)) # project multiple points without error
        """
        # in case of point2D = list([u, v])
        if isinstance(point2D, list):
            point2D = np.array(point2D)
        if point2D.ndim == 1:
            point2D = point2D[:, np.newaxis]
        assert point2D.shape[0] == 2
        point2D = point2D.astype(np.float32)
        undistorted_points = cv2.undistortPoints(point2D.transpose(), self._camera_matrix, self._dist_coeffs)
        cam_3d = undistorted_points.reshape(undistorted_points.shape[0], undistorted_points.shape[2])
        cam_3d = np.hstack((cam_3d, np.ones((undistorted_points.shape[0], 1))))
        # self.print_np(self.cam_3d)
        cam_3d /= np.linalg.norm(cam_3d, axis=1, keepdims=True)  # normalize to unit norm
        return cam_3d.transpose()

    def __world2cam(self, point3D):
        """ __world2cam(point3D) projects a 3D point on to the image.
        If points are projected on the outside of the fov, return (-1,-1).
        Also, return (-1, -1), if point (x, y, z) = (0, 0, 0).
        The coordinate is different than that of the original OcamCalib.
        point3D coord: x:right direction, y:down direction, z:front direction
        point2D coord: x:row direction, y:col direction (OpenCV image coordinate).

        Parameters
        ----------
        point3D : numpy array or list([x, y, z])
            array of points in camera coordinate (3xN)

        Returns
        -------
        point2D : numpy array
            array of points in image (2xN)

        Examples
        --------
        ocam.__world2cam([1,1,2.0]).tolist() # project a point on image
        [[1004.8294677734375], [1001.1594848632812]]
        tmp = ocam.__world2cam(np.random.rand(3, 10)) # project multiple points without error
        ocam.__world2cam([0,0,2.0]).tolist() # return optical center
        [[798.1757202148438], [794.3086547851562]]
        ocam.__world2cam([0,0,0]).tolist()
        [[-1.0], [-1.0]]
        """
        # in case of point3D = list([x,y,z])
        if isinstance(point3D, list):
            point3D = np.array(point3D)
        if point3D.ndim == 1:
            point3D = point3D[:, np.newaxis] # to 3X1
        assert point3D.shape[0] == 3
        # return value
        point3D = point3D.astype(np.float32)
        point2D = np.zeros((2, point3D.shape[1]), dtype=np.float32)
        point2D, _ = cv2.projectPoints(point3D, self._rotation, self._translation,
                                       self._camera_matrix, self._dist_coeffs)
        point2D = point2D.reshape(point2D.shape[0], point2D.shape[2]).transpose()
        return point2D

    def __hash__(self):
        return hash(self.__repr__())

    def __eq__(self, other):
        return self.__repr__() == other.__repr__()


# params_yaml = CameraWideParameters()
# params_yaml.read_from_yaml_file(r"params/intrinsic_rear_wide_kbfisheye.yaml")
# ocam = CameraWideModel()
# ocam.set_parameters(params_yaml)

# img = cv2.imread(r"1.jpg")
# mapx, mapy = ocam.undist2plane_perspective()
# img_out = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
# # Visualize images
# cv2.imwrite("2.jpg", img_out)
# img_out = cv2.resize(img_out, (0, 0), fx=0.5, fy=0.5)  # 显示0.5倍大小
# cv2.imshow("undistort", img_out)
# cv2.waitKey()


# cor = np.array([[1798, 684], [1743, 628], [1736, 611], [1735, 600],
#                 [1739, 598], [1740, 598], [1746, 587]])
# # print(cor.shape[0])
# # for i in range(cor.shape[0]):
# #     d = ocam._CameraScaraModel__cam2world(cor[i,:] )
# #     print(cor[i,:], ": ", d.T)
#
# # c = np.array([0.994465, 0.0718484, -0.0766626])
# d = ocam._CameraWideModel__cam2world(cor.T)
# print(d.T)
# c = d
#
# print(c)
# # c = np.random.rand(3, 10)
# # print(np.linalg.norm(c))
# d = ocam._CameraWideModel__world2cam(c)
# print(d)
# print(d.T.astype('int'))