'''
 # Copyright 2024 ZF. All Rights Reserved.
 # Author:
 #        xuheng.chai@zf.com
 #####################################################################
 # @file
 # @brief Camera Scaramuzza parameters class.
'''
import yaml
from .zf_camera_base_parameters import CameraBaseParameters


class CameraScaraParameters(CameraBaseParameters):
    """ Camera Scaramuzza Parameters class.
    """
    def __init__(self):
        super().__init__()
        # polynomial coefficients for the DIRECT mapping function
        self._pol = []
        # polynomial coefficients for the inverse mapping function
        self._invpol = []
        # _affine parameters "c", "d", "e"
        self._affine = []
        self.__is_data_valid = False

    def is_param_valid(self):
        return self.__is_data_valid

    def read_from_yaml_file(self, filename, print_flag=False):
        self.__is_data_valid = False
        with open(filename, "r") as file:
            yaml_data = yaml.safe_load(file)
        width = yaml_data["image size"]["width"]
        height = yaml_data["image size"]["height"]
        cx = yaml_data["distortion center"]["cx"]
        cy = yaml_data["distortion center"]["cy"]
        c = yaml_data["affine coefficients"]["c"]
        d = yaml_data["affine coefficients"]["d"]
        e = yaml_data["affine coefficients"]["e"]
        length_pol = yaml_data["camera2world"]["length_pol"]
        pol = yaml_data["camera2world"]["pol"]
        length_invpol = yaml_data["world2camera"]["length_invpol"]
        invpol = yaml_data["world2camera"]["invpol"]
        self._pol = pol
        self._invpol = invpol
        self._xc = cy
        self._yc = cx
        self._affine = [c, d, e]
        self._img_height = height
        self._img_width = width
        self.__is_data_valid = True
        if print_flag:
            print(self)

    def get_hw(self):
        return self._img_height, self._img_width

    def __repr__(self):
        print_list = []
        print_list.append(f"pol: {self._pol}")
        print_list.append(f"invpol: {self._invpol}")
        print_list.append(f"xc(col dir): {self._xc}, \tyc(row dir): {self._yc} in Ocam coord")
        print_list.append(f"affine: {self._affine}")
        print_list.append(f"img_height: {self._img_height}, \timg_width: {self._img_width}")
        return "\n".join(print_list)