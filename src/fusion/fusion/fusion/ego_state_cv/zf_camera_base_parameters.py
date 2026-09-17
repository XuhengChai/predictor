'''
 # Copyright 2024 ZF. All Rights Reserved.
 # Author:
 #        xuheng.chai@zf.com
 #####################################################################
 # @file
 # @brief Camera Base Parameters class..
'''
from enum import Enum, unique

@unique
class ECameraModelType(Enum):
    SCARAMUZZA = "Scaramuzza"
    KANNALA_BRANDT = "kb"
    MEI = "Mei"

@unique
class EPatternType(Enum):
    CHESSBOARD = "chessboard"
    CIRCLES_GRID = "circles"

class CameraBaseParameters():
    """ Camera Base Parameters class.
    """
    def __init__(self):
        self._model_type = ECameraModelType.SCARAMUZZA       # default type is Scaramuzza
        # image size: "height" and "width"
        self._img_width = 500
        self._img_height = 500
        # center: "row" and "column", starting from 0 (C convention)
        self._xc = 0
        self._yc = 0

    def read_from_yaml_file(self, filename, print_flag=False):
        pass

    def read_from_txt_file(self, filename, print_flag=False):
        pass

    @property
    def camera_model_type(self):
        return self._model_type

    @property
    def width(self):
        """ Getter for image width."""
        return self._img_width

    @property
    def height(self):
        """ Getter for image height."""
        return self._img_height

    @property
    def cx(self):
        """ Getter for image center cx (OpenCV format)."""
        return self._yc

    @property
    def cy(self):
        """ Getter for image center cy (OpenCV format)."""
        return self._xc

    def __repr__(self):
        print_list = []
        print_list.append(f"xc(col dir): {self._xc}, \tyc(row dir): {self._yc} in Ocam coord")
        print_list.append(f"img_height: {self._img_height}, \timg_width: {self._img_width}")
        return "\n".join(print_list)

    def __hash__(self):
        return hash(self.__repr__())

    def __eq__(self, other):
        return self.__repr__() == other.__repr__()