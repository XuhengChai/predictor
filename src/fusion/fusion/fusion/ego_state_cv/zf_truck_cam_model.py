'''
 # Copyright 2024 ZF. All Rights Reserved.
 # Author:
 #        xuheng.chai@zf.com
 #####################################################################
 # @file
 # @brief Camera model of truck.
 Calculate ground distance according to the pixel on image captured by camera on truck.
'''
import copy
import os
import cv2
import numpy as np
from .zf_camera_scara_model import CameraScaraModel
from .zf_camera_scara_parameters import CameraScaraParameters
from .zf_camera_wide_model import CameraWideModel
from .zf_camera_wide_parameters import CameraWideParameters
from .zf_coordinate_transform import CoordinateTransform, ECameraIntrExtr


class TruckCameraModel:
    """
        unit: m
        camera_param_lr: left or right camera
            LEFT_INTR
            RIGHT_INTR
    """
    def __init__(self, camera_param_lr: ECameraIntrExtr = ECameraIntrExtr.RIGHT_INTR):
        current_folder = os.path.dirname(os.path.realpath(__file__))
        file_name_dic: dict[ECameraIntrExtr, str] = {}
        file_name_dic[ECameraIntrExtr.LEFT_IN_VEHICLE] \
            = os.path.join(current_folder, r"params/extrinsic_left_front_fisheye_to_vehicle_ext.yaml")
        file_name_dic[ECameraIntrExtr.LEFT_IN_FISHEYE] \
            = os.path.join(current_folder, r"params/extrinsic_vehicle_to_left_front_fisheye_ext.yaml")
        file_name_dic[ECameraIntrExtr.RIGHT_IN_VEHICLE] \
            = os.path.join(current_folder, r"params/extrinsic_right_front_fisheye_to_vehicle_ext.yaml")
        file_name_dic[ECameraIntrExtr.RIGHT_IN_FISHEYE] \
            = os.path.join(current_folder, r"params/extrinsic_vehicle_to_right_front_fisheye_ext.yaml")
        file_name_dic[ECameraIntrExtr.LEFT_INTR] \
            = os.path.join(current_folder, "params/intrinsic_left_front_fisheye_ocam.yaml")
        file_name_dic[ECameraIntrExtr.RIGHT_INTR] \
            = os.path.join(current_folder, "params/intrinsic_right_front_fisheye_ocam.yaml")
        file_name_dic[ECameraIntrExtr.WIDE_IN_VEHICLE] \
            = os.path.join(current_folder, r"params/extrinsic_front_wide_to_vehicle_ext.yaml")
        file_name_dic[ECameraIntrExtr.VEHICLE_IN_WIDE] \
            = os.path.join(current_folder, r"params/extrinsic_vehicle_to_front_wide_ext.yaml")
        file_name_dic[ECameraIntrExtr.WIDE] \
            = os.path.join(current_folder, r"params/intrinsic_rear_wide_kbfisheye.yaml")

        if camera_param_lr == ECameraIntrExtr.WIDE_IN_VEHICLE \
                or camera_param_lr == ECameraIntrExtr.VEHICLE_IN_WIDE \
                or camera_param_lr == ECameraIntrExtr.WIDE:
            self._camera_type = ECameraIntrExtr.WIDE
            self._params_yaml = CameraWideParameters()
            self._params_yaml.read_from_yaml_file(file_name_dic[self._camera_type])
            self._ocam = CameraWideModel()
            self._ocam.set_parameters(self._params_yaml)
            self.__mapx, self.__mapy = self._ocam.undist2plane_perspective()
        else:
            self._camera_type = ECameraIntrExtr.RIGHT_INTR
            if camera_param_lr == ECameraIntrExtr.LEFT_INTR \
                    or camera_param_lr == ECameraIntrExtr.LEFT_IN_VEHICLE \
                    or camera_param_lr == ECameraIntrExtr.LEFT_IN_FISHEYE:
                self._camera_type = ECameraIntrExtr.LEFT_INTR
            self._params_yaml = CameraScaraParameters()
            self._params_yaml.read_from_yaml_file(file_name_dic[self._camera_type])
            self._ocam = CameraScaraModel()
            self._ocam.set_parameters(self._params_yaml)
            self.__mapx, self.__mapy = self._ocam.undist2plane_polyconic()
        self._trans = CoordinateTransform(param_paths=file_name_dic)
        self._trans.set_cam_type(self._camera_type)
        self._zc = -0.7
        self._zero_coord = np.array([0, 0, 1])
        self._zero_angle = 0.0
        self._zero_length = 1.0
        self._zero_height = 0.0
        self._init_zero_angle = False
        self.__img_h, self.__img_w = self._params_yaml.get_hw() # 1080, 1920

    def print_np(self, M):
        M_str = np.array2string(M, separator=',', formatter={'float': lambda x: "%.8f" % x})
        print("np.array([", M_str[1:-1], "])")

    def get_hw(self):
        return self.__img_h, self.__img_w

    """
        param:  zc & is_in_world: refer point height in world frame / in camera frame
                type: float
    """
    def set_zc(self, zc: float, is_in_world: bool = False):
        if is_in_world:
            t_point = self._trans.get_P_ground2cameraRectify([0, 0, zc])
            zc = t_point[2]
        self._zc = zc

    def get_mapxy(self):
        return self.__mapx, self.__mapy

    def limit_number(self, num, minv, maxv):
        if num < minv:
            return minv
        if num > maxv:
            return maxv
        return num

    def undistort_pixel2origin(self, x, y):
        tx = int(y)
        ty = int(x)
        return [self.limit_number(self.__mapx[tx][ty], 0, self.__img_w),
                self.limit_number(self.__mapy[tx][ty], 0, self.__img_h)]

    """
        param:  pixel when ego state angle is 0 & is_origin_img: point (u, v) in origin image frame / in undistort image frame
                type: numpy array 1X2/2X1 matrix or list
    """
    def set_zero_angle_pixel(self, pixel, is_origin_img: bool = True):
        p_ground = self.pixel2d_2_ground3d(pixel, is_origin_img, self._zc)
        self._zero_angle = np.degrees(np.arctan2(p_ground[1], p_ground[0]))
        self._zero_coord = p_ground
        self._zero_length = np.linalg.norm(p_ground[:2])
        self._zero_height = p_ground[2]
        self._init_zero_angle = True

    def set_zero_length(self, _len):
        self._zero_length = _len

    def set_zero_height(self, height):
        self._zero_height = height

    """
        param: pixel & is_origin_img: point (u, v) in origin image frame / in undistort image frame
               type: numpy array 1X2/2X1 matrix or list
    """
    def cal_ego_state_angle(self, pixel, is_origin_img: bool = True):
        p_ground = self.pixel2d_2_ground3d(pixel, is_origin_img, self._zc)
        p_ground_len = np.linalg.norm(p_ground[:2])
        zero_coord_modify = self._modify_coord_by_len(p_ground_len)
        ego_angle = np.degrees(np.arctan2(p_ground[1], p_ground[0]) - np.arctan2(zero_coord_modify[1], zero_coord_modify[0]))
        return ego_angle
        
    """
        return: point_in_ground: point in ground frame (centre is on ground)
                type: numpy array 3X1 matrix
    """
    def pixel2d_2_ground3d(self, pixel, is_distort_img: bool = True, zc: float = np.NAN):
        p2d = pixel
        p3d = np.array([])
        if is_distort_img:
            p3d = self._ocam.frame_pixel2camera(p2d)
        else:
            p2d_ori = self.undistort_pixel2origin(pixel[0], pixel[1])
            p3d = self._ocam.frame_pixel2camera(p2d_ori)
        p3d_rectify = self._trans.get_P_camera2rectify(p3d)
        if zc is np.NAN:
            zc = self._zc
        norm_len = zc / p3d_rectify[2]
        p3d_rectify *= norm_len
        p_ground = self._trans.get_P_cameraRectify2ground(p3d_rectify)
        return p_ground[:3]

    def ground3ds_2_pixel2ds_f(self, pos3d, cam_name: ECameraIntrExtr = ECameraIntrExtr.RIGHT_INTR):
        # pos4d: 4Xn, p4d: 4Xn
        ones_row = np.ones((1, pos3d.shape[1]))
        pos4d = np.concatenate((pos3d, ones_row), axis=0)
        # pos4d[2] = 1.45762  # height of trailer to ground
        # pos4d[2] = 0
        p4ds = self._trans.ground2camera(pos4d)
        p2d = self._ocam.frame_camera2pixel(p4ds[:3, :])
        return p2d.transpose()# nX2

    def pixel2d_2_camera3d(self, pixel, is_distort_img: bool = True, zc: float = np.NAN):
        p2d = pixel
        p3d = np.array([])
        if is_distort_img:
            p3d = self._ocam.frame_pixel2camera(p2d)
        else:
            p2d_ori = self.undistort_pixel2origin(pixel[0], pixel[1])
            p3d = self._ocam.frame_pixel2camera(p2d_ori)
        # p3d_rectify = self._trans.get_P_camera2rectify(p3d, self._camera_type)
        # p3d_rectify = self._trans.get_P_camera2rectify(p3d, self._camera_type)
        return p3d

    """
        param:  angle: degrees
                type: float
        return: pixel point: point in camera frame
                type: numpy array 2X1 matrix
     """
    
    def cal_pixel_from_angle(self, angle, is_degree: bool = True):
        if not self._init_zero_angle:
            print("Warning: init zero angle first")
            return
        if is_degree:
            angle = np.deg2rad(angle + self._zero_angle)
        else:
            angle = angle + np.deg2rad(self._zero_angle)
        x = self._zero_length * np.cos(angle)
        y = self._zero_length * np.sin(angle)
        p3d_ground = [x, y, self._zero_height]
        p3d_camera = self._trans.get_P_ground2camera(p3d_ground, self._camera_type)
        p3d_camera /= np.linalg.norm(p3d_camera, axis=0)
        p2d = self._ocam.frame_camera2pixel(p3d_camera)
        return p2d

    def cal_muti_pixel_from_angle(self, angle, muti_len:list, is_absolute: bool = False, is_degree: bool = True):
        if not self._init_zero_angle:
            print("Warning: init zero angle first")
            return
        if is_degree:
            angle = np.deg2rad(angle)
        size = len(muti_len)
        point_array = np.ones((4, size))
        len_array = np.array(muti_len)
        if not is_absolute:
            len_array += self._zero_length
        for i in range(size):
            zero_coord_modify = self._modify_coord_by_len(len_array[i])
            angle_modify = angle + np.arctan2(zero_coord_modify[1], zero_coord_modify[0])
            point_array[0, i] = len_array[i] * np.cos(angle_modify) # X-coordinate
            point_array[1, i] = len_array[i] * np.sin(angle_modify)  # Y-coordinate
            point_array[2, i] = self._zero_height  # Z-coordinate
        p3ds_camera = self._trans.ground2camera(point_array)
        p3ds_camera[:3, :] /= np.linalg.norm(p3ds_camera[:3, :], axis=0)
        p2d = self._ocam.frame_camera2pixel(p3ds_camera[:3, :])
        return p2d

    def _modify_coord_by_len(self, length):
        zero_coord_modify = copy.deepcopy(self._zero_coord)
        if length <= abs(zero_coord_modify[1]):
            return
        zero_coord_modify[0] = np.sign(zero_coord_modify[0]) * np.sqrt(length**2 - zero_coord_modify[1]**2)
        return zero_coord_modify

class PlotOnImgBase(TruckCameraModel):
    """
        unit: m
        camera_param_lr: left or right camera
            LEFT_INTR
            RIGHT_INTR
    """
    def __init__(self, camera_param_lr: ECameraIntrExtr = ECameraIntrExtr.RIGHT_INTR):
        super().__init__(camera_param_lr)
        self.img = None
        self.CV_COLOR_RED = (0, 0, 255, 255)
        self.CV_COLOR_GREEN = (0, 255, 0, 255)
        self.CV_COLOR_YELLOW = (0,255,255, 255)
        self.CV_COLOR_BLUE = (255, 0, 0, 255)
        self.CV_COLOR_ORANGERED = (0, 69, 255, 255)  # 橙红色
        self.CV_COLOR_CHOCOLATE = (30, 105, 210, 255)  # 巧克力
        self.CV_COLOR_GOLD = (10, 215, 255, 255)  # 金色
        self.CV_COLOR_DEEPPINK = (147, 20, 255, 255)  # 深粉色
        self.CV_COLOR_VIOLET = (238, 130, 238, 255)  # 紫罗兰

    def ground3ds_2_pixel2ds(self, pos3d, cam_name: ECameraIntrExtr = ECameraIntrExtr.RIGHT_INTR):
        # pos4d: 4Xn, p4d: 4Xn
        ones_row = np.ones((1, pos3d.shape[1]))
        pos4d = np.concatenate((pos3d, ones_row), axis=0)
        # pos4d[2] = 1.45762  # height of trailer to ground
        # pos4d[2] = 0
        p4ds = self._trans.ground2camera(pos4d)
        p2d = self._ocam.frame_camera2pixel(p4ds[:3, :])
        # return p2d.transpose().astype('int')# nX2
        # return np.rint(p2d.transpose()).astype('int')# nX2
        return p2d.transpose()# nX2

    def get_radar_pixel(self, pos3d: list):
        # p3d = self._trans.get_P_radar2camera([pos3d[1],pos3d[0],0], radar_name)
        p3d = self._trans.get_P_radar2camera(pos3d)
        p2d = self._ocam.frame_camera2pixel(p3d)
        return p2d.flatten().astype('int')

    def z_rotation_matrix2d(self, angle, is_degree=True):
        """
        Returns a 3D transformation matrix for a rotation about the z-axis
        by a given angle in either degrees or radians.

        Parameters:
        angle (float): The rotation angle.
        is_degree (bool): True if the angle is in degrees, False if it is in radians.
                          Default is False.

        Returns:
        A 3x3 NumPy array representing the transformation matrix.
        """
        if is_degree:
            angle = np.deg2rad(angle)
        cos = np.cos(angle)
        sin = np.sin(angle)
        return np.array([[cos, -sin, 0],
                         [sin, cos, 0],
                         [0, 0, 1]])

    def draw_text_line(self, img, point, text_line: str, drawType="custom"):
        '''
        :param img:
        :param point:
        :param text:
        :param drawType: custom or custom
        :return:
        '''
        fontScale = 0.4
        thickness = 5
        fontFace = cv2.FONT_HERSHEY_SIMPLEX
        # fontFace=cv2.FONT_HERSHEY_SIMPLEX
        text_line = text_line.split("\n")
        # text_size, baseline = cv2.getTextSize(str(text_line), fontFace, fontScale, thickness)
        text_size, baseline = cv2.getTextSize(str(text_line), fontFace, fontScale, thickness)
        for i, text in enumerate(text_line):
            if text:
                draw_point = [point[0], point[1] + (text_size[1] + 10 + baseline) * i]
                cv2.putText(img, text, draw_point, cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,0,255), 1,
                        cv2.LINE_AA)
        return img

    def set_img(self, img):
        self.img = img

    def save_img(self, save_path):
        cv2.imwrite(save_path, self.img,[int(cv2.IMWRITE_JPEG_QUALITY),50])