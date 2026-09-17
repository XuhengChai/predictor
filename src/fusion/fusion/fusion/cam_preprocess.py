from enum import Enum, unique, IntEnum
import numpy as np
import math

import time
import cv2
from numpy import random
# import openpyxl
# import pandas as pd
# from pandas.core.frame import DataFrame
# from shapely.geometry import Point
# from shapely.geometry.polygon import Polygon
import os

from .ego_state_cv.zf_truck_cam_model import TruckCameraModel, ECameraIntrExtr

dimensions_set = {
    0: (0.8, 0.5, 1.8),  # person
    1: (1.6, 0.3, 1.865),  # biycle
    2: (4.075, 1.668, 1.474),  # car
    3: (1.75, 0.5, 1.0),  # motorcycle
    5: (9.6, 2.5, 3.45),  # bus
    7: (7.2, 2.3, 2.7),  # truck
    6: (0.5, 0.5, 0.5),  #
    4: (0.5, 0.5, 0.5),  #
    8: (0.5, 0.5, 0.5),  #
    9: (0.5, 0.5, 0.5),  #
    10: (0.5, 0.5, 0.5)  #
}

@unique
class EPropBox(IntEnum):
    lt_x = 0
    lt_y = 1
    rb_x = 2
    rb_y = 3
    id_track = 4
    conf = 5
    cls = 6
    idx = 7
    wx = idx + 1
    wy = wx + 1
    wz = wy + 1
    head = wz + 1
    cp_x = head + 1
    cp_y = cp_x + 1
    length = cp_y + 1
    width = length + 1
    origin_rect = width + 1
    head_revise = origin_rect + 1
    scale = head_revise + 1
    scale_head = scale + 1
    head_rotate = scale_head + 1
    img_len = head_rotate + 1

class CamPrePrc:
    def __init__(self):
        self.cam_r = TruckCameraModel(ECameraIntrExtr.RIGHT_INTR)
        self.cam_r.set_zc(0, True)
        self.class_name = {
            0: 'pedestrian',
            1: 'bicycle',
            2: 'car',
            3: 'motorbike',
            5: 'bus',
            7: 'truck',
            10: 'person_OGM',
            11: 'bicycle_OGM',
            12: 'car_OGM',
            13: 'motorcycle_OGM',
            15: 'bus_OGM',
            17: 'truck_OGM'
        }

    # [642.798828125, 259.339111328125, 665.6474609375, 309.8814697265625, 2.0, 0.5567852258682251, 0.0]
    def process_boxes(self, boxes, ego_angle=0.0):
        result_dict = {}
        # for box in boxes:
        for i in range(len(boxes)-1, -1, -1):
            if boxes[i][EPropBox.rb_x] > 1910:
                boxes.pop(i)
                continue
            if len(boxes[i]) != 8:
                boxes[i].append(1)
                if len(boxes[i]) != 8:
                    continue
            box = boxes[i]
            self.cal_props(box)
            box[EPropBox.id_track] = int(box[EPropBox.id_track])
            # self.cal_props_simple(box)
            result_dict[box[EPropBox.id_track]] = {
                'class': self.class_name[int(box[EPropBox.cls])],
                'class_confidence': box[EPropBox.conf],
                'pos_x': box[EPropBox.cp_x],
                'pos_y': box[EPropBox.cp_y],
                'width': box[EPropBox.width],
                'length': box[EPropBox.length],
                'heading': box[EPropBox.head] + box[EPropBox.head_revise],
                # 'flag': 1,
                'bbox': [box[EPropBox.lt_x], box[EPropBox.lt_y], box[EPropBox.rb_x], box[EPropBox.rb_y]]
            }
        return result_dict

    # 1~2
    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x))

    def angle_2vec(self, v0, v1):
        return np.arctan2(np.linalg.det([v0, v1]), np.dot(v0, v1))

    def limit_number(self, sign_num, min_val, max_val):
        num = abs(sign_num)
        proportion = (num - min_val)/(max_val - min_val)
        scale = 8*proportion - 4
        sig = self.sigmoid(scale)
        num = min_val + (max_val - min_val)*sig
        num2 = max_val - (max_val - min_val)*sig
        return np.sign(sign_num)*num, scale, np.sign(sign_num)*num2

    def revise_coordinate(self, p0):
        p0[0] -= 4.75
        p0[1] += 1
        for i in range(len(p0)):
            # coeff = 1/((p0[i]/10) ** 2 + 1) # 1/(x^2+1)
            # p0[i] *= coeff
            # t = self.sigmoid(p0[i]/10)
            # coeff = t*(1-t)*4 # 1/(x^2+1)
            if abs(p0[i]/2) > 4:
                p0[i] = np.sign(p0[i])*math.log2(abs(p0[i]/2))*4
        # ([42.39457036718984, -8.224302723181813], [35.958426906872745, -13.177282880402123])
        # p_3D_lb, p_3D_rb
        # ([16.93747955555612, -7.224302723181813], [15.855454933749275, -10.424481420812395])

    def log2_x(self, x):
        if x < 2:
            return x/2
        return math.log2(x)

    def revise_2coordinate(self, p0, p1):
        p0[0] -= 4.75
        p0[1] += 1
        p1[0] -= 4.75
        p1[1] += 1
        vec_origin = [p1[1] - p0[1], p1[0] - p0[0]]
        min_len_origin = self.log2_x(np.linalg.norm(np.array(vec_origin)))

        for i in range(len(p0)):
            # coeff = 1/((p0[i]/10) ** 2 + 1) # 1/(x^2+1)
            # p0[i] *= coeff
            # t = self.sigmoid(p0[i]/10)
            # coeff = t*(1-t)*4 # 1/(x^2+1)
            if abs(p0[i]/2) > 4:
                p0[i] = np.sign(p0[i])*math.log2(abs(p0[i]/2))*4
            if abs(p1[i]/2) > 4:
                p1[i] = np.sign(p1[i])*math.log2(abs(p1[i]/2))*4
        vec = [p1[1] - p0[1], p1[0] - p0[0]]
        min_len = self.log2_x(np.linalg.norm(np.array(vec)))
        min_coeff = min_len_origin / min_len
        return min_coeff


    def get_coeff(self, x, x_range, y_range):
        x0 = x_range[0]
        x1 = x_range[1]
        y0 = y_range[0]
        y1 = y_range[1]
        x_var = (x1 - x0) / 2
        y_var = (y1 - y0) / 2
        t = x_var / 4
        #     head_revise = ((self.sigmoid((box[EPropBox.cp_x] + 12.5) / 3) - 0.5)*0.85 + 1.575) * head_revise
        coeff = (self.sigmoid((x - x0 - x_var) / t) - 0.5) * 2 * y_var + y0 + y_var
        return coeff

    def cal_props_simple(self, box:list):  # pixels: two bottom bbox points in image
        if dimensions_set[box[EPropBox.cls]]:
            l, w, _ = dimensions_set[box[EPropBox.cls]]
        else:
            l, w, _ = 1.0, 1.0, 1.0
        box[EPropBox.lt_x] = self.cam_r.limit_number(box[EPropBox.lt_x], 0, 1919)
        box[EPropBox.lt_y] = self.cam_r.limit_number(box[EPropBox.lt_y], 0, 1079)
        box[EPropBox.rb_x] = self.cam_r.limit_number(box[EPropBox.rb_x], 0, 1919)
        box[EPropBox.rb_y] = self.cam_r.limit_number(box[EPropBox.rb_y], 0, 1079)
        center_pixel = [round((box[EPropBox.lt_x] + box[EPropBox.rb_x]) / 2),
                        round((box[EPropBox.rb_y]))]
        # box.extend(center_pixel)
        point_3D_world = self.cam_r.pixel2d_2_ground3d(center_pixel, True)  # np.array
        # p_3D = point_3D_world[0:2].flatten().tolist()  # Vehicle world coordinate:x to the top, y to the left
        p_3D = point_3D_world.flatten().tolist()  # Vehicle world coordinate:x to the top, y to the left
        box.extend(p_3D)

    def cal_props(self, box: list):  # pixels: two bottom bbox points in image
        if dimensions_set[box[EPropBox.cls]]:
            l, w, _ = dimensions_set[box[EPropBox.cls]]
        else:
            l, w, _ = 1.0, 1.0, 1.0
        length, width = l, w
        is_2w = (box[EPropBox.cls] == 1 or box[EPropBox.cls] == 3)
        box[EPropBox.lt_x] = self.cam_r.limit_number(box[EPropBox.lt_x], 0, 1919)
        box[EPropBox.lt_y] = self.cam_r.limit_number(box[EPropBox.lt_y], 0, 1079)
        box[EPropBox.rb_x] = self.cam_r.limit_number(box[EPropBox.rb_x], 0, 1919)
        box[EPropBox.rb_y] = self.cam_r.limit_number(box[EPropBox.rb_y], 0, 1079)
        center_pixel = [round((box[EPropBox.lt_x] + box[EPropBox.rb_x]) / 2),
                        round((box[EPropBox.rb_y]))]
        # box.extend(center_pixel)
        point_3D_world = self.cam_r.pixel2d_2_ground3d(center_pixel, False)  # np.array
        # p_3D = point_3D_world[0:2].flatten().tolist()  # Vehicle world coordinate:x to the top, y to the left
        p_3D = point_3D_world.flatten().tolist()  # Vehicle world coordinate:x to the top, y to the left
        box.extend(p_3D)

        p_3D_lb = self.cam_r.pixel2d_2_ground3d([box[EPropBox.lt_x], box[EPropBox.rb_y]], False)[0:2].flatten().tolist()
        p_3D_rb = self.cam_r.pixel2d_2_ground3d([box[EPropBox.rb_x], box[EPropBox.rb_y]], False)[0:2].flatten().tolist()
        # if abs(box[EPropBox.wx] - 3.83) < 20:
        # lr_vec_len, _, _ = self.limit_number(np.linalg.norm(np.array([p_3D_lb[0] - p_3D_rb[0],
        #           p_3D_lb[1] - p_3D_rb[1]])), w, l)

        min_coeff = self.revise_2coordinate(p_3D_lb, p_3D_rb)

        heading = math.atan2(p_3D_lb[1] - p_3D_rb[1], p_3D_lb[0] - p_3D_rb[0])
        # if is_2w and p_3D[0] > -5 and p_3D[0] < 3:
        #     min_coeff = min_coeff/abs(math.cos(heading))
        if is_2w and p_3D[0] > 4 and p_3D[0] < 7:
            min_coeff = min_coeff*(abs(math.cos(heading)) - 0.2)
            if p_3D[0] < 6:
                w += 0.3
        box.append(heading)
        R = self.z_rotation_matrix2d(heading, False)
        # print(top_left, top_right, bottom_left, bottom_right)
        lr_vec = [(p_3D_lb[0] - p_3D_rb[0])*min_coeff,
                  (p_3D_lb[1] - p_3D_rb[1])*min_coeff]
        lr_vec_len, scale, lr_len_reverse = self.limit_number(np.linalg.norm(np.array(lr_vec)), w, l)


        p_3D_center = R @ (np.array([0, -lr_len_reverse / 2, 0]).reshape(3, 1))
        p_3D_center_small = R @ (np.array([0, -w / 2, 0]).reshape(3, 1))
        # p_3D_center = R @ (np.array([0, -w/2, 0]).reshape(3, 1))
        box.append((p_3D_center[0] + p_3D[0])[0])
        box.append((p_3D_center_small[1] + p_3D[1])[0])
        box.append(length)
        box.append(width)
        origin_pixels = []
        origin_pixels.append(self.cam_r.undistort_pixel2origin(box[EPropBox.lt_x], box[EPropBox.lt_y]))
        origin_pixels.append(self.cam_r.undistort_pixel2origin(box[EPropBox.rb_x], box[EPropBox.lt_y]))
        origin_pixels.append(self.cam_r.undistort_pixel2origin(box[EPropBox.rb_x], box[EPropBox.rb_y]))
        origin_pixels.append(self.cam_r.undistort_pixel2origin(box[EPropBox.lt_x], box[EPropBox.rb_y]))
        # 0   1
        #   /
        # /
        # 3   2 form point 2 to point 1
        pixels = np.array(origin_pixels)
        dia_line_min = np.min(pixels, axis=0) # Maxima along the first axis col
        dia_line_max = np.max(pixels, axis=0)
        v0 = np.array([dia_line_max[0] - dia_line_min[0], dia_line_min[1] - dia_line_max[1]])
        v31 = pixels[1] - pixels[3]
        v1 = np.array([dia_line_max[0] - dia_line_min[0], dia_line_max[1] - dia_line_min[1]])
        v02 = pixels[2] - pixels[0]
        # head_revise = (self.sigmoid((box[EPropBox.cp_x] + 7.5) / 3) + 1)*(self.angle_2vec(v0, v31) + self.angle_2vec(v1, v02))/2
        head_revise = self.angle_2vec(v0, v31) + self.angle_2vec(v1, v02)
        head_rotate = head_revise
        # if box[EPropBox.cp_x] > 5.83: # 2 + 3.83
        #     head_revise = 2 * head_revise
        # elif box[EPropBox.cp_x] < 4.5: # 0.64 + 3.83
        #     # -20 ~ 0.5 1.15 ~ 2
        #     head_revise = ((self.sigmoid((box[EPropBox.cp_x] + 12.5) / 3) - 0.5)*0.85 + 1.575) * head_revise

        if box[EPropBox.cp_x] > 4.5 and box[EPropBox.cp_x] < 20: # 2 + 3.83
            if box[EPropBox.cp_x] < 12:
                head_revise = self.get_coeff(box[EPropBox.cp_x], [4.5, 10], [0.5, 0.75]) * head_revise
            else:
                head_revise = self.get_coeff(box[EPropBox.cp_x], [4.5, 20], [0.5, 1]) * head_revise
        # elif box[EPropBox.cp_x] < 4.5: # 0.64 + 3.83
        #     # -20 ~ 0.5 1.15 ~ 2
        #     head_revise = head_revise
        # if box[EPropBox.cp_x] > 4.5: # 15.18 + 3.83
        #     head_revise = ((-self.sigmoid((box[EPropBox.cp_x] - 7.5) / 2) - 0.5)*0.68 + 0.84) * head_revise
        # elif box[EPropBox.cp_x] < 4.5: # 0.64 + 3.83
        #     # -20 ~ 0.5 1.15 ~ 2
        #     head_revise = ((-self.sigmoid((box[EPropBox.cp_x] + 15) / 4) + 0.5)*0.5 + 1.25) * head_revise
        # head_revise = (np.arctan2(np.linalg.det([v0, v1]), np.dot(v0, v1)))

        box.append(origin_pixels)
        # box.append(head_revise)
        # print(top_left, top_right, bottom_left, bottom_right)
        scale_head = np.sign(head_revise)*(l - lr_vec_len)/(l - w)*np.pi/2
        if abs(scale) > 1:
            angle = (l - lr_vec_len)/(l - w)*np.pi/2
            if scale < 0:
                if scale < -4:
                    head_revise = np.sign(head_revise) * angle
                else:
                    coeff = self.get_coeff(scale, [-4, -1], [1, 0])
                    head_revise = np.sign(head_revise) * angle * coeff + (1 - coeff) * head_revise
            elif scale > 3.2:
                head_revise = np.sign(head_revise)*angle
            else:
                coeff = self.get_coeff(scale, [1.5, 3.2], [0, 1])
                head_revise = np.sign(head_revise)*angle*coeff + (1-coeff)*head_revise

        box.append(head_revise)
        box.append(scale)
        box.append(scale_head)
        box.append(head_rotate)
        box.append(np.linalg.norm(np.array(lr_vec)))





        # return box  # return: 目标类别，中心点坐标，四个角点坐标，长，宽，方位的单位向量
        # [[1073.1897015041436, 226.56964061093998, 1122.895995140177, 261.640317001136, 2.0, 0.7298414707183838, 2.0,
        #   -1.0801793432566562, -20.704304673555868, -0.32192548454716424, array([-0.81631]), array([-19.913]), 4.075,
        #   1.668],

    def z_rotation_matrix2d(self, angle, is_degree=True):
        if is_degree:
            angle = np.deg2rad(angle)
        cos = np.cos(angle)
        sin = np.sin(angle)
        return np.array([[cos, -sin, 0],
                         [sin, cos, 0],
                         [0, 0, 1]])