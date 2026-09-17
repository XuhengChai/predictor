'''
 # Copyright 2024 ZF. All Rights Reserved.
 # Author:
 #        xuheng.chai@zf.com
 #####################################################################
 # @file
 # @brief Camera Scaramuzza model class.
'''

import numpy as np
from .zf_camera_base_model import CameraBaseModel
from .zf_camera_scara_parameters import CameraScaraParameters


class CameraScaraModel(CameraBaseModel):
    """ OCamCalib[1] unndistortion class.
    ----------Parameters
    fov : float
        field of view of the camera in degree
    ----------References
    [1] https://sites.google.com/site/scarabotix/ocamcalib-toolbox
    """

    def __init__(self, fov=360):
        self._fov = fov

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

    def set_parameters(self, param: CameraScaraParameters):
        if (not param.is_param_valid()):
            raise Exception("invalid parameters, please read the model config file first!")
        self._param = param
        self._img_width = param.width
        self._img_height = param.height
        self._pol = param._pol  # polynomial coefficients for the DIRECT mapping function
        self._invpol = param._invpol    # polynomial coefficients for the inverse mapping function
        # center: "row" and "column", starting from 0 (C convention)
        self._xc = param._xc
        self._yc = param._yc
        self._affine = param._affine    # _affine parameters "c", "d", "e"

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

    def frame_undistort_pixel2origin(self, point2D):
        H = self._img_height
        W = self._img_width
        th = np.pi / H
        p = 2 * np.pi / W
        arrayW = point2D[0]
        arrayH = point2D[1]
        phi =  (arrayW + 0.5) * p - np.pi
        theta = (arrayH + 0.5) * th - np.pi / 2
        phi_xy, theta_xy = np.meshgrid(phi, theta, sparse=False, indexing="xy")
        point3D = np.stack(
            [np.sin(phi_xy) * np.cos(theta_xy), np.sin(theta_xy), np.cos(phi_xy) * np.cos(theta_xy)]).reshape(3, -1)
        point2D_ori = self.__world2cam(point3D)
        print("point2D_ori: ",  point2D_ori)
        return point2D_ori

    # Polyconic projection
    def undist2plane_polyconic(self, optimized=True):
        if (not self._param):
            raise Exception('no parameters, please read the model config file first!')
        if (not self._param.is_param_valid()):
            raise Exception('invalid parameters, please read the model config file first!')
        H = self._img_height
        W = self._img_width
        ys, xs = np.indices((H, W), np.float32)  # np.pi/5
        # pa = [1, 0.3, 0, 1.6, 0.2, 0, 1.6, 1.5, 1.4, -0.7, 15]
        # pa = [1, 0.0, 0, 1.6, 0.2, 0, 1.6, 1.5, 1.4, -0.7, 15]
        # 0.3, 0 linspace  1.6:x_proj  0.2, 0 linspace
        pa = [1, 0.1, 0, 1.5, 0.1, 0, 1.35, 1.45, 1.6, -0.6, 0, 15, 0]
        if optimized:
            pa = [1, 0.1, 0, 1.5, 0.1, 0, 1.55, 1.45, 1.45, -0.6, 0.05, 15, 2]
        phi0 = pa[-2] / 180 * np.pi
        lambda0 = pa[-1] / 180 * np.pi
        y_proj = (1 - (ys) / H) * pa[0]
        x = np.linspace(0, np.pi / 2, H)
        y = np.sin(x)
        # sample H = 1080 points
        sampled_indices = np.linspace(0, len(x) - 1, H).astype(int)
        x1d = np.linspace(pa[1], pa[2], H)
        deta_x2d = np.tile(pa[3] + y[sampled_indices] * x1d, (W, 1)).transpose()
        x_proj = (xs - W / 2) / W * deta_x2d #- 0.02
        A = phi0 + y_proj
        B = x_proj ** 2 + A ** 2
        phi = A
        for i in range(6):
            deta_den = (phi - A) / np.tan(phi) - 1
            deta_phi = - (A * (phi * np.tan(phi) + 1) - phi - 1 / 2 * (phi * phi + B) * np.tan(phi)) / deta_den
            phi = phi + deta_phi
        deta_y1d = np.linspace(pa[4], pa[5], W)
        deta_y2d = np.tile(deta_y1d, (H, 1))
        phi_alt = (1 - phi) * pa[6] + deta_y2d
        theta_alt = np.arcsin(x_proj * np.tan(phi)) / np.sin(phi) + lambda0
        theta_alt = theta_alt * pa[7]
        phi_alt = phi_alt * pa[8] + pa[9]
        xs = 1
        if optimized:
            x = np.linspace(np.pi / 3, 2 * np.pi / 3, W)
            y = np.sin(x)
            # sample W = 1920 points
            sampled_indices = np.linspace(0, len(x) - 1, W).astype(int)
            xs = np.tile(y[sampled_indices] * pa[8], (H, 1))
        x = np.sin(theta_alt) * np.cos(phi_alt) * xs
        y = np.sin(phi_alt)
        z = np.cos(theta_alt) * np.cos(phi_alt) + pa[10]
        point3D = np.stack([x, y, z]).reshape(3, -1)
        mapx, mapy = self.__world2cam(point3D)
        mapx = mapx.reshape(H, W)
        mapy = mapy.reshape(H, W)
        return mapx, mapy

    # Equirectangular projection
    def undist2plane(self):
        if (not self._param):
            raise Exception("no parameters, please read the model config file first!")
        if (not self._param.is_param_valid()):
            raise Exception("invalid parameters, please read the model config file first!")
        H = self._img_height
        W = self._img_width
        th = np.pi / H
        p = 2 * np.pi / W
        arrayW = np.linspace(0, W - 1, W)
        arrayH = np.linspace(0, H - 1, H)
        phi = (arrayW + 0.5) * p - np.pi
        theta = (arrayH + 0.5) * th - np.pi / 2
        phi_xy, theta_xy = np.meshgrid(phi, theta, sparse=False, indexing="xy")
        point3D = np.stack(
            [np.sin(phi_xy) * np.cos(theta_xy), np.sin(theta_xy), np.cos(phi_xy) * np.cos(theta_xy)]).reshape(3, -1)
        mapx, mapy = self.__world2cam(point3D)
        mapx = mapx.reshape(H, W)
        mapy = mapy.reshape(H, W)
        return mapx, mapy

    # Perspective projection
    def undist2plane_perspective(self, f=1.0):
        if (not self._param):
            raise Exception("no parameters, please read the model config file first!")
        if (not self._param.is_param_valid()):
            raise Exception("invalid parameters, please read the model config file first!")
        H = self._img_height
        W = self._img_width
        focal = abs(f)
        focal = 0.2 if focal < 0.2 else focal
        z = W / focal
        arrayW = np.linspace(0, W - 1, W)
        arrayH = np.linspace(0, H - 1, H)
        x = arrayW - W / 2  
        y = arrayH - H / 2
        x_grid, y_grid = np.meshgrid(x, y, sparse=False, indexing="xy")
        t = np.stack([x_grid, y_grid, np.full_like(x_grid, z)])
        point3D = np.stack([x_grid, y_grid, np.full_like(x_grid, z)]).reshape(3, -1)
        mapx, mapy = self.__world2cam(point3D)
        mapx = mapx.reshape(H, W)
        mapy = mapy.reshape(H, W)
        return mapx, mapy

    # M: 3X3 matrix
    def undist2plane_bev(self, M:np.array = np.NAN, fov=10):
        if (not self._param):
            raise Exception('no parameters, please read the model config file first!')
        if (not self._param.is_param_valid()):
            raise Exception('invalid parameters, please read the model config file first!')

        H = self._img_height
        W = self._img_width
        # fov = 10
        if M is np.NAN:
            M = np.array([[3.25358679e-01, -6.89051449e-01, 5.93062013e+02],
                          [-5.87948257e-02, -2.84274759e-01, 4.88433284e+02],
                          [-6.05425658e-05, -7.40327129e-04, 1.00000000e+00]])
        z = W / fov
        x = np.linspace(0, W - 1, W)  # 以中间为0，[-w/2, w/2),1递增
        y = np.linspace(0, H - 1, H)
        x_grid, y_grid = np.meshgrid(x, y, sparse=False, indexing='xy')
        x_flat = x_grid.flatten()
        y_flat = y_grid.flatten()
        points = np.vstack((x_flat, y_flat, np.ones_like(x_flat)))
        # Perform perspective transformation for all points
        transformed_points = np.dot(M, points)
        # Normalize the resulting homogeneous coordinate vectors
        transformed_points /= transformed_points[2]
        # Reshape the transformed points back to matrices
        x_grid = np.reshape(transformed_points[0], x_grid.shape).astype(np.float32) - W / 2
        y_grid = np.reshape(transformed_points[1], y_grid.shape).astype(np.float32) - H / 2
        point3D = np.stack([x_grid, y_grid, np.full_like(x_grid, z)]).reshape(3, -1)
        mapx, mapy = self.__world2cam(point3D)
        mapx = mapx.reshape(H, W)
        mapy = mapy.reshape(H, W)
        return mapx, mapy

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
        >>> ocam.__cam2world([502,900]).tolist() # project a point onto unit sphere
        [[-0.5776824148317081], [0.20599312860435134], [0.7898416667674589]]
        >>> tmp = ocam.__cam2world(1600*np.random.rand(2, 10)) # project multiple points without error
        """
        # in case of point2D = list([u, v])
        if isinstance(point2D, list):
            point2D = np.array(point2D)
        if point2D.ndim == 1:
            point2D = point2D[:, np.newaxis]
        assert point2D.shape[0] == 2
        invdet = 1 / (self._affine[0] - self._affine[1] * self._affine[2])
        xp = invdet * ((point2D[1] - self._xc) - self._affine[1] * (point2D[0] - self._yc))
        yp = invdet * (-self._affine[2] * (point2D[1] - self._xc) + self._affine[0] * (point2D[0] - self._yc))
        r = np.sqrt(xp * xp + yp * yp)  # distance [pixels] of  the point from the image center
        # be careful about z axis direction
        # zp = -np.array([element * r ** i for (i, element) in enumerate(self._pol)]).sum(axis=0) is slow
        for (i, element) in enumerate(self._pol):
            if i == 0:
                zp = np.full_like(r, element)
                tmp_r = r.copy()
            else:
                zp += element * tmp_r
                tmp_r *= r
        zp *= -1
        point3D = np.stack([yp, xp, zp])
        point3D /= np.linalg.norm(point3D, axis=0)  # normalize to unit norm
        return point3D

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
        >>> ocam.__world2cam([1,1,2.0]).tolist() # project a point on image
        [[1004.8294677734375], [1001.1594848632812]]
        >>> tmp = ocam.__world2cam(np.random.rand(3, 10)) # project multiple points without error
        >>> ocam.__world2cam([0,0,2.0]).tolist() # return optical center
        [[798.1757202148438], [794.3086547851562]]
        >>> ocam.__world2cam([0,0,0]).tolist()
        [[-1.0], [-1.0]]
        """
        # in case of point3D = list([x,y,z])
        if isinstance(point3D, list):
            point3D = np.array(point3D)
        if point3D.ndim == 1:
            point3D = point3D[:, np.newaxis] # to 3X1
        assert point3D.shape[0] == 3
        # return value
        point2D = np.zeros((2, point3D.shape[1]), dtype=np.float32)
        norm = np.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1])
        valid_flag = (norm != 0)
        # optical center
        point2D[0][~valid_flag] = self._yc
        point2D[1][~valid_flag] = self._xc
        zero_flag = (point3D == 0).all(axis=0)
        point2D[0][zero_flag] = -1
        point2D[1][zero_flag] = -1
        # else
        theta = -np.arctan(point3D[2][valid_flag] / norm[valid_flag])
        invnorm = 1 / norm[valid_flag]
        #     rho = np.array([element * theta ** i for (i, element) in enumerate(self._invpol)]).sum(axis=0) is slow
        for (i, element) in enumerate(self._invpol):
            if i == 0:
                rho = np.full_like(theta, element)
                tmp_theta = theta.copy()
            else:
                rho += element * tmp_theta
                tmp_theta *= theta
        u = point3D[0][valid_flag] * invnorm * rho
        v = point3D[1][valid_flag] * invnorm * rho
        point2D_valid_0 = v * self._affine[2] + u + self._yc
        point2D_valid_1 = v * self._affine[0] + u * self._affine[1] + self._xc
        if self._fov < 360:
            # finally deal with points are outside of fov
            thresh_theta = np.deg2rad(self._fov / 2) - np.pi / 2
            # set flag when  or point3D == (0, 0, 0)
            outside_flag = theta > thresh_theta
            point2D_valid_0[outside_flag] = -1
            point2D_valid_1[outside_flag] = -1
        point2D[0][valid_flag] = point2D_valid_0
        point2D[1][valid_flag] = point2D_valid_1
        return point2D

    def __repr__(self):
        print_list = []
        print_list.append(f"pol: {self._pol}")
        print_list.append(f"invpol: {self._invpol}")
        print_list.append(f"xc(col dir): {self._xc}, \tyc(row dir): {self._yc} in Ocam coord")
        print_list.append(f"affine: {self._affine}")
        print_list.append(f"img_size: {self._img_size}")
        if self._fov < 360:
            print_list.append(f"fov: {self._fov}")
        return "\n".join(print_list)

    def __hash__(self):
        return hash(self.__repr__())

    def __eq__(self, other):
        return self.__repr__() == other.__repr__()