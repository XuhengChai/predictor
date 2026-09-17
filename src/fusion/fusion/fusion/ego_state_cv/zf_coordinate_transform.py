'''
 # Copyright 2024 ZF. All Rights Reserved.
 # Author:
 #        xuheng.chai@zf.com
 #####################################################################
 # @file
 # @brief Coordinate transform among camera frame, ground frame and radar frame.
'''
import copy
from enum import Enum, unique
import numpy as np
import transformations as tf
import yaml


@unique
class ERadarName(Enum):
    LEFT_FRONT = "radar_left_front_in_ground"
    LEFT_BACK = "radar_left_back_in_ground"
    RIGHT_FRONT = "radar_right_front_in_ground"
    RIGHT_BACK = "radar_right_back_in_ground"


@unique
class ECameraIntrExtr(Enum):  # define the class ECameraIntrExtr with Enum type
    LEFT_IN_VEHICLE = "left_fisheye_in_vehicle"
    LEFT_IN_FISHEYE = "left_vehicle_in_fisheye"
    RIGHT_IN_VEHICLE = "right_fisheye_in_vehicle"
    RIGHT_IN_FISHEYE = "right_vehicle_in_fisheye"
    WIDE_IN_VEHICLE = "wide_in_vehicle"
    VEHICLE_IN_WIDE = "vehicle_in_wide"
    WIDE = "wide"
    LEFT_INTR = "left_intrinsic"
    RIGHT_INTR = "right_intrinsic"
    TXT_INTR = "intrinsic"


class CoordinateTransform:
    """  Coordinate Transform class, from radar and camera, to vehicle and to pixel.
    """
    def __init__(self, param_paths):  # param_paths: dict[ECameraIntrExtr, str]
        self._param_left_fisheye_in_vehicle = param_paths[ECameraIntrExtr.LEFT_IN_VEHICLE]
        self._param_left_vehicle_in_fisheye = param_paths[ECameraIntrExtr.LEFT_IN_FISHEYE]
        self._param_right_fisheye_in_vehicle = param_paths[ECameraIntrExtr.RIGHT_IN_VEHICLE]
        self._param_right_vehicle_in_fisheye = param_paths[ECameraIntrExtr.RIGHT_IN_FISHEYE]

        self._param_wide_in_vehicle = param_paths[ECameraIntrExtr.WIDE_IN_VEHICLE]
        self._param_vehicle_in_wide = param_paths[ECameraIntrExtr.VEHICLE_IN_WIDE]
        self._param_left_intrinsic_front_fisheye = param_paths[ECameraIntrExtr.LEFT_INTR]
        self._param_right_intrinsic_front_fisheye = param_paths[ECameraIntrExtr.RIGHT_INTR]
        self._in_ground_T_radar = {}
        self.get_T_radar_in_ground()
        self._in_ground_T_camera_ = {}
        self._in_camera_T_ground_ = {}
        self.init_T_camera_and_ground()
        self.cam_name = ECameraIntrExtr.RIGHT_IN_VEHICLE
        self.cam_name_reverse = ECameraIntrExtr.RIGHT_IN_FISHEYE

    def set_cam_type(self, camera):
        if camera == ECameraIntrExtr.LEFT_INTR \
                or camera == ECameraIntrExtr.LEFT_IN_VEHICLE \
                or camera == ECameraIntrExtr.LEFT_IN_FISHEYE:
            self.cam_name = ECameraIntrExtr.LEFT_IN_VEHICLE
            self.cam_name_reverse = ECameraIntrExtr.LEFT_IN_FISHEYE
        if camera == ECameraIntrExtr.RIGHT_INTR \
                or camera == ECameraIntrExtr.RIGHT_IN_VEHICLE \
                or camera == ECameraIntrExtr.RIGHT_IN_FISHEYE:
            self.cam_name = ECameraIntrExtr.RIGHT_IN_VEHICLE
            self.cam_name_reverse = ECameraIntrExtr.RIGHT_IN_FISHEYE
        if camera == ECameraIntrExtr.WIDE_IN_VEHICLE \
                or camera == ECameraIntrExtr.VEHICLE_IN_WIDE \
                or camera == ECameraIntrExtr.WIDE:
            self.cam_name = ECameraIntrExtr.WIDE_IN_VEHICLE
            self.cam_name_reverse = ECameraIntrExtr.VEHICLE_IN_WIDE


    """
        param:
            T44: numpy array 4X4 matrix
        return:
            inv : numpy array 4X4 matrix
    """
    def cal_inv_matrix(self, T44) -> np.ndarray:
        R = T44[:3, :3]
        t = T44[:3, 3]
        Rinv = np.transpose(R)
        tinv = -1 * Rinv @ t

        inv = np.eye(4)
        inv[:3, :3] = Rinv
        inv[:3, 3] = tinv
        return inv

    def get_transfrom_matrix(self, file_path: str) -> np.ndarray:
        with open(file_path, "r") as file:
            parameters = yaml.safe_load(file)
        # Extract the values from the parameters dictionary
        x = parameters["x"]
        y = parameters["y"]
        z = parameters["z"]
        qw = parameters["qw"]
        qx = parameters["qx"]
        qy = parameters["qy"]
        qz = parameters["qz"]
        yaw = parameters["yaw"]
        pitch = parameters["pitch"]
        roll = parameters["roll"]
        rotation_matrix = tf.euler_matrix(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw),
                                          "sxyz")  # Create the rotation matrix from Euler angles
        rotation_matrix[:3, 3] = [x, y, z]
        quaternion_matrix = tf.quaternion_matrix([qw, qx, qy, qz])
        quaternion_matrix[:3, 3] = [x, y, z]
        return quaternion_matrix

    def set_h_radar_in_ground(self, h):
        tire_T_in_ground = np.eye(4)
        tire_T_in_ground[:3, 3] = [3.830, 0, h] # front tire
        self._in_ground_T_radar[ERadarName.LEFT_FRONT] = tire_T_in_ground
        self._in_ground_T_radar[ERadarName.LEFT_BACK] = tire_T_in_ground
        self._in_ground_T_radar[ERadarName.RIGHT_FRONT] = tire_T_in_ground
        self._in_ground_T_radar[ERadarName.RIGHT_BACK] = tire_T_in_ground

    def get_T_radar_in_ground(self):
        self.set_h_radar_in_ground(0.505)

    def init_T_camera_and_ground(self):
        self._in_ground_T_camera_[ECameraIntrExtr.LEFT_IN_VEHICLE] \
            = self.get_transfrom_matrix(self._param_left_fisheye_in_vehicle)
        self._in_ground_T_camera_[ECameraIntrExtr.RIGHT_IN_VEHICLE] \
            = self.get_transfrom_matrix(self._param_right_fisheye_in_vehicle)
        self._in_ground_T_camera_[ECameraIntrExtr.WIDE_IN_VEHICLE] \
            = self.get_transfrom_matrix(self._param_wide_in_vehicle)

        self._in_camera_T_ground_[ECameraIntrExtr.LEFT_IN_FISHEYE] \
            = self.get_transfrom_matrix(self._param_left_vehicle_in_fisheye)
        self._in_camera_T_ground_[ECameraIntrExtr.RIGHT_IN_FISHEYE] \
            = self.get_transfrom_matrix(self._param_right_vehicle_in_fisheye)
        self._in_camera_T_ground_[ECameraIntrExtr.VEHICLE_IN_WIDE] \
            = self.get_transfrom_matrix(self._param_vehicle_in_wide)

    """
        param:  point: point in radar frame
                type:   numpy array 1X3/3X1 matrix 
        return: point_in_camera: point in camera frame
                type:   numpy array 4X1 matrix
    """
    def get_P_radar2camera(self, point, radar: ERadarName = ERadarName.RIGHT_FRONT):
        point41 = np.array([point[0], point[1], point[2], 1], dtype=float).reshape(4, 1)
        point_in_camera = np.array([])
        if radar == ERadarName.LEFT_FRONT or radar == ERadarName.LEFT_BACK:
            point_in_camera = self._in_camera_T_ground_[ECameraIntrExtr.LEFT_IN_FISHEYE] \
                              @ self._in_ground_T_radar[radar] @ point41
        if radar == ERadarName.RIGHT_FRONT or radar == ERadarName.RIGHT_BACK:
            point_in_camera = self._in_camera_T_ground_[ECameraIntrExtr.RIGHT_IN_FISHEYE] \
                              @ self._in_ground_T_radar[radar] @ point41
        return point_in_camera[0:3]

    def get_P_radar2cameraRectify(self, point, radar: ERadarName = ERadarName.RIGHT_FRONT):
        point41 = np.array([point[0], point[1], point[2], 1], dtype=float).reshape(4, 1)
        _in_rectify_camera_T_ground = np.array([])
        if radar == ERadarName.LEFT_FRONT or radar == ERadarName.LEFT_BACK:
            _in_rectify_camera_T_ground = copy.deepcopy(self._in_ground_T_camera_[ECameraIntrExtr.LEFT_IN_VEHICLE])
        if radar == ERadarName.RIGHT_FRONT or radar == ERadarName.RIGHT_BACK:
            _in_rectify_camera_T_ground = copy.deepcopy(self._in_ground_T_camera_[ECameraIntrExtr.RIGHT_IN_VEHICLE])
        _in_rectify_camera_T_ground[:3, :3] = np.eye(3)
        _in_rectify_camera_T_ground[:3, 3] *= -1
        point_in_camera_rectify = _in_rectify_camera_T_ground @ self._in_ground_T_radar[radar] @ point41
        return point_in_camera_rectify[0:3]

    def get_P_cameraRectify2radar(self, point, radar: ERadarName = ERadarName.RIGHT_FRONT):
        point41 = np.array([point[0], point[1], point[2], 1], dtype=float).reshape(4, 1)
        _in_ground_T_rectify_camera = np.array([])
        if radar == ERadarName.LEFT_FRONT or radar == ERadarName.LEFT_BACK:
            _in_ground_T_rectify_camera = copy.deepcopy(self._in_ground_T_camera_[ECameraIntrExtr.LEFT_IN_VEHICLE])
        if radar == ERadarName.RIGHT_FRONT or radar == ERadarName.RIGHT_BACK:
            _in_ground_T_rectify_camera = copy.deepcopy(self._in_ground_T_camera_[ECameraIntrExtr.RIGHT_IN_VEHICLE])
        _in_ground_T_rectify_camera[:3, :3] = np.eye(3)
        point_in_rectify_camera = self.cal_inv_matrix(self._in_ground_T_radar[radar]) \
                                  @ _in_ground_T_rectify_camera @ point41
        return point_in_rectify_camera

    def get_P_ground2cameraRectify(self, point):
        point41 = np.array([point[0], point[1], point[2], 1], dtype=float).reshape(4, 1)
        _in_rectify_camera_T_ground = np.array([])
        _in_rectify_camera_T_ground = copy.deepcopy(self._in_ground_T_camera_[self.cam_name])
        _in_rectify_camera_T_ground[:3, :3] = np.eye(3)
        _in_rectify_camera_T_ground[:3, 3] *= -1
        point_in_rectify_camera = _in_rectify_camera_T_ground @ point41
        return point_in_rectify_camera

    def get_P_cameraRectify2ground(self, point):
        point41 = np.array([point[0][0], point[1][0], point[2][0], 1], dtype=float).reshape(4, 1)
        _in_ground_T_rectify_camera = np.array([])
        _in_ground_T_rectify_camera = copy.deepcopy(self._in_ground_T_camera_[self.cam_name])
        _in_ground_T_rectify_camera[:3, :3] = np.eye(3)
        point_in_ground = _in_ground_T_rectify_camera @ point41
        return point_in_ground

    def get_P_camera2rectify(self, point):
        point31 = np.array([point[0], point[1], point[2]], dtype=float).reshape(3, 1)
        R = self._in_ground_T_camera_[self.cam_name][:3, :3]
        point = R @ point31
        return point

    def get_P_rectify2camera(self, point):
        point31 = np.array([point[0], point[1], point[2]], dtype=float).reshape(3, 1)
        R = self._in_ground_T_camera_[self.cam_name][:3, :3]
        point = R.transpose() @ point31
        return point

    def get_P_camera2radar(self, point, radar: ERadarName = ERadarName.RIGHT_FRONT):
        point41 = np.array([point[0], point[1], point[2], 1], dtype=float).reshape(4, 1)
        point_in_radar = np.array([])
        if radar == ERadarName.LEFT_FRONT or radar == ERadarName.LEFT_BACK:
            point_in_radar = self.cal_inv_matrix(self._in_ground_T_radar[radar]) \
                             @ self._in_ground_T_camera_[ECameraIntrExtr.LEFT_IN_VEHICLE] @ point41
        if radar == ERadarName.RIGHT_FRONT or radar == ERadarName.RIGHT_BACK:
            point_in_radar = self.cal_inv_matrix(self._in_ground_T_radar[radar]) \
                             @ self._in_ground_T_camera_[ECameraIntrExtr.RIGHT_IN_VEHICLE] @ point41
        return point_in_radar[0:3]

    def get_P_radar2ground(self, point, radar: ERadarName = ERadarName.RIGHT_FRONT):
        point41 = np.array([point[0], point[1], point[2], 1], dtype=float).reshape(4, 1)
        point = np.array([])
        if radar == ERadarName.LEFT_FRONT or radar == ERadarName.LEFT_BACK:
            point = self._in_ground_T_radar[radar] @ point41
        if radar == ERadarName.RIGHT_FRONT or radar == ERadarName.RIGHT_BACK:
            point = self._in_ground_T_radar[radar] @ point41
        return point[0:3]

    def get_P_ground2radar(self, point, radar: ERadarName = ERadarName.RIGHT_FRONT):
        point41 = np.array([point[0], point[1], point[2], 1], dtype=float).reshape(4, 1)
        point = np.array([])
        if radar == ERadarName.LEFT_FRONT or radar == ERadarName.LEFT_BACK:
            point = self.cal_inv_matrix(self._in_ground_T_radar[radar]) @ point41
        if radar == ERadarName.RIGHT_FRONT or radar == ERadarName.RIGHT_BACK:
            point = self.cal_inv_matrix(self._in_ground_T_radar[radar]) @ point41
        return point[0:3]

    def get_P_ground2camera(self, point, camera: ECameraIntrExtr):
        point41 = np.array([point[0], point[1], point[2], 1], dtype=float).reshape(4, 1)
        points = self._in_camera_T_ground_[self.cam_name_reverse] @ point41
        return point[0:3]

    def ground2camera(self, points4n):
        points = self._in_camera_T_ground_[self.cam_name_reverse] @ points4n
        return points

    def get_P_camera2ground(self, point):
        point41 = np.array([point[0], point[1], point[2], 1], dtype=float).reshape(4, 1)
        point = self._in_ground_T_camera_[self.cam_name] @ point41
        return point[0:3]
