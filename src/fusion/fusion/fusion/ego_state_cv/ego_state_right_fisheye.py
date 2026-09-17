import cv2
import numpy as np

from .zf_coordinate_transform import ECameraIntrExtr
from .zf_truck_cam_model import TruckCameraModel


class FindEgoStateAngle():
    def __init__(self):
        self.reset_init()
        self._state = "Init"     # means if the EgoStateAngle module trigger (Enable) or not (Disable)

    def reset_init(self):
        self._zero_pixel_list = []
        self._r_list = [0.0,  -1.0, -2.0, -3.0, -4.0, -5.0]
        self._right_ego = TruckCameraModel(ECameraIntrExtr.RIGHT_INTR)
        self._img_raw = None
        self._img_gray = None
        self._cur_angle = 0.0
        self._min_threshold = 1.0
        self._cur_threshold = 1.0
        self._zero_gradient_list = []       # for nearst gradient
        self._gradient_weight = np.array([])
        self._gradient_threshold = 1
        self._gradient_threshold_change = 0
        self._zero_gradient = None          # for max gradient
        self._angle_pixel = {}
        self._init_right_ego()
        self._confidence = 0.0              # articulation angle's confidence: cal. in def cal_near_gradient_pixel(self)
        self._angle_pixels = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]


    def _init_right_ego(self):
        zc = -0.7
        p2d_right_0 = [1780, 512]           # Pixel coordinate values of the right-rear-bottom corner of the trailer
        self._right_ego.set_zc(zc)
        self._right_ego.set_zero_angle_pixel(p2d_right_0)
        p2d_new = self._right_ego.cal_muti_pixel_from_angle(0.0, self._r_list)
        self._zero_pixel_list = np.round(p2d_new.transpose()).astype(int)

    def get_roi(self, pixel, size=5):
        return self._img_gray[int(pixel[1] - size):int(pixel[1] + size), int(pixel[0] - size):int(pixel[0] + size)]

    def cal_angle_dic(self, angle, threshold, num=10):
        low = angle - threshold
        if low < 0:
            low = 0
        high = angle + threshold
        angles = np.linspace(low, high, num)
        p2d = {}
        for i in range(0, angles.shape[0]):
            p2d_new = self._right_ego.cal_muti_pixel_from_angle(angles[i], self._r_list)
            p2d_list = p2d_new.transpose()
            p2d[angles[i]] = np.round(p2d_list).astype(int).tolist()

        return p2d      # return dict: {float angle: [[x1,y1], [x2,y2] ...]}

    def cal_roi_gradient(self, pixel, size=5):
        roi = self.get_roi(pixel)
        dst = cv2.cornerHarris(roi, blockSize=2, ksize=3, k=0.04)       # return the Reponse value of Harris Algorithm for each point. dst.shape == img_gray.shape
        row = roi.shape[0]
        col = roi.shape[1]
        weight = np.zeros((row, col))       # Create an array of zeros
        mid_start = col // 2 - 1            # Starting index of the middle columns
        mid_end = mid_start + 2 if col % 2 == 0 else mid_start + 3      # Ending index of the middle columns
        weight[:, mid_start:mid_end] = 1
        weight[:, :mid_start] = 0.5
        weight[:, mid_end:] = 0.5
        weight[:, 0] = 0.1
        weight[:, -1] = 0.1
        weight[:, 2] = 0.25
        weight[:, -2] = 0.25
        dst = weight * dst

        return 0.5 * np.max(dst) + np.mean(dst)

    def cal_max_gradient_pixel(self):
        angle_dic = self.cal_angle_dic(self._cur_angle, self._cur_threshold)
        gradient_dic = {}
        for key, val in angle_dic.items():
            gradient_list = []
            for pixel in val:
                gradient_list.append(self.cal_roi_gradient(pixel))

            gradient_dic[key] = np.mean(self._gradient_weight * gradient_list)
        
        max_key = max(gradient_dic, key=lambda x: gradient_dic[x])
        if not self._zero_gradient:
            self._zero_gradient = gradient_dic[max_key]
        if abs(max_key - 0.0) < 0.01:
            self._zero_gradient = (gradient_dic[max_key] + self._zero_gradient)/2
        if gradient_dic[max_key] > 1.5 * self._zero_gradient or gradient_dic[max_key] < 0.25 * self._zero_gradient:
            return self._angle_pixel
        
        return {max_key: angle_dic[max_key]}

    def cal_near_gradient_pixel(self):
        angle_dic = self.cal_angle_dic(self._cur_angle, self._cur_threshold)
        gradient_dic = {}
        gradient_list_dic = {}
        for key, val in angle_dic.items():
            gradient_list = []
            for pixel in val:
                gradient_list.append(self.cal_roi_gradient(pixel))
            gradient_list_dic[key] = np.array(gradient_list)
            err_list = abs(np.array(gradient_list)/np.array(self._zero_gradient_list)
                           + np.array(self._zero_gradient_list)/np.array(gradient_list)) - 2

            gradient_dic[key] = np.mean(self._gradient_weight * abs(err_list))  

        max_key = min(gradient_dic, key=lambda x: gradient_dic[x])
        self._confidence = 2.0 - 2.0/(1.0 + np.exp(-gradient_dic[max_key]*0.5))
        self._confidence = round(self._confidence, 3)

        if abs(max_key - self._cur_angle) < 0.5 and gradient_dic[max_key] < 0.1:
            self._zero_gradient_list = 0.25*gradient_list_dic[max_key] + 0.75*np.array(self._zero_gradient_list)
        
        if gradient_dic[max_key] > 10:
            self._cur_threshold += 0.5
            if (self._cur_threshold > 3):
                self._cur_threshold = 3
            return self._angle_pixel
        else:
            if abs(abs(max_key - self._cur_angle) - self._cur_threshold) > 0.01:
                self._cur_threshold -= 0.5
            else:
                self._cur_threshold += 0.5
            if (self._cur_threshold < self._min_threshold):
                self._cur_threshold = self._min_threshold
        
        return {max_key: angle_dic[max_key]}

    def angle_cal(self, raw_img, fusion_angle):
        self._img_raw = raw_img
        self._img_gray = cv2.cvtColor(self._img_raw, cv2.COLOR_BGR2GRAY)
        if not len(self._zero_gradient_list):
            for pixel in self._zero_pixel_list:
                self._zero_gradient_list.append(self.cal_roi_gradient(pixel))

            a_list = np.array(range(len(self._zero_gradient_list))) + 1
            self._gradient_weight = a_list / np.linalg.norm(a_list)
            self._gradient_weight[0] = self._gradient_weight[2]
            self._gradient_weight[2] = self._gradient_weight[-3]

        self._angle_pixel = self.cal_near_gradient_pixel()      #   print(self._angle_pixel):{angle: [[u1, v1], [u2, v2], [u3, v3], [u4, v4], [u5, v5], [u6, v6]]}
        artic_angle = list(self._angle_pixel.keys())[0]         #   angle is storaged as keys of self._angle_pixel
        #self._cur_angle = artic_angle                           #   Update the self._cur_angle with new calculated angle
        self._cur_angle = fusion_angle   
        self._angle_pixels = self._angle_pixel[artic_angle]
        artic_angle = round(artic_angle, 3)

        return  artic_angle     #   unit: degree
