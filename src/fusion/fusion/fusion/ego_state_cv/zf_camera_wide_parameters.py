'''
 # Copyright 2024 ZF. All Rights Reserved.
 # Author:
 #        xuheng.chai@zf.com
 #####################################################################
 # @file
 # @brief Camera Scaramuzza parameters class.
'''
import yaml
import numpy as np
from .zf_camera_base_parameters import CameraBaseParameters


class CameraWideParameters(CameraBaseParameters):
    """ Camera Scaramuzza Parameters class.
    """
    def __init__(self):
        super().__init__()
        self._camera_matrix = np.identity(3)
        self._dist_coeffs = np.zeros(4).reshape(4, 1)
        self.__is_data_valid = False

    def is_param_valid(self):
        return self.__is_data_valid

    def read_from_yaml_file(self, filename, print_flag=False):
        self.__is_data_valid = False
        with open(filename, "r") as file:
            yaml_data = yaml.safe_load(file)
        width = yaml_data["width"]
        height = yaml_data["height"]
        cx = yaml_data["cx"]
        cy = yaml_data["cy"]
        fx = yaml_data["fx"]
        fy = yaml_data["fy"]
        skew = yaml_data["skew"]

        self._camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)
        dist_coeffs = np.array(
            [yaml_data["k1"], yaml_data["k2"],
             yaml_data["k3"], yaml_data["k4"]])
        self._dist_coeffs = dist_coeffs.reshape(4, 1)
        self._img_height = height
        self._img_width = width
        self.__is_data_valid = True
        if print_flag:
            print(self)

    def get_hw(self):
        return self._img_height, self._img_width

    def __repr__(self):
        print_list = []
        print_list.append(f"camera_matrix: {self._camera_matrix}")
        print_list.append(f"dist_coeffs: {self._dist_coeffs}")
        print_list.append(f"img_height: {self._img_height}, \timg_width: {self._img_width}")
        return "\n".join(print_list)

# a = CameraWideParameters()
# a.read_from_yaml_file(r"params/intrinsic_rear_wide_kbfisheye.yaml")
# print(a)