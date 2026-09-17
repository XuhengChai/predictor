# -*- coding: utf-8 -*-
"""
Created on Mon Mar 11 11:13:11 2024

@author: ZHANG Jing
"""
import re
import ast
import os
import pickle
import time
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from PIL import Image
from collections import deque
from shapely.geometry import box
from shapely import affinity
from scipy.spatial import distance_matrix
import math
import sys
# ============================================================================
# Online ROS
# from fusion.utility.kmfilter_class import KMFilter
# from fusion.cam_post_detection import Processing as CV_preproc
# from fusion.utility.plt_preprc_snsr import PlotBEV
# from fusion.analyze_cv_data import Plot as plot_overall_pic
# ============================================================================
# Offline Test
from fusion.utility.kmfilter_class import KMFilter
# from post_detection_12_26 import Processing as CV_preproc
# from utility.plt_preprc_snsr import PlotBEV
# from analyze_cv_data import Plot as plot_overall_pic
# import line_profiler
# pd.options.mode.chained_assignment=None
# ============================================================================




# %% data reading functions -> not used for later online processing
def read_cam_frames(data_folder):
    files = os.listdir(data_folder[0])
    cv_fname = data_folder[1] + ".pkl"
    cv_path = os.path.join(data_folder[0], cv_fname)
    proc_cv_fname = data_folder[1] + "_proc.pkl"
    proc_cv_path = os.path.join(data_folder[0], proc_cv_fname)
    ego_cv_fname = data_folder[1] + "_ego.pkl"
    ego_cv_path = os.path.join(data_folder[0], ego_cv_fname)
    if proc_cv_fname in files:
        with open(proc_cv_path, "rb") as f:
            cam_frames = pickle.load(f)
    elif cv_fname in files:
        with open(cv_path, "rb") as f:
            cam_frames = pickle.load(f)
            cam_frames = process_track_result(cam_frames)
            with open(proc_cv_path,"wb") as f:
                pickle.dump(cam_frames, f)
    if ego_cv_fname in files:
        with open(ego_cv_path, "rb") as f:
            ego_states = pickle.load(f)
    else:
        ego_states = pd.DataFrame([[0.0] * len(cam_frames),
                                   [0.0] * len(cam_frames)]).T
        ego_states.columns = ["veh_spd", "str_angle"]
    # cam_frames = list(cam_frames.values())
    return cam_frames, ego_states


def process_track_result(cam_frames):
    res_frames = {}
    a = CV_preproc()
    i = 0
    for frame in sorted(cam_frames.keys()):
        dt = cam_frames[frame]
        ts = frame
        res = a.offline_cv_processing([list(x)[:-1] for x in list(dt)])
        res_frames[ts] = res
        i+=1
    return res_frames


def modify_cam_class(cam_frames):
    obj_class = {"person": "pedestrian",
                 "bicycle": "bicycle",
                 "car": "car",
                 "motorcycle": "motorbike",
                 "bus": "bus",
                 "truck": "truck"}
    if isinstance(cam_frames, list):
        for df in cam_frames:
            if len(df) > 0:
                df["class"] = df["class"].apply(lambda x: obj_class[x])
    elif isinstance(cam_frames, dict):
        for k in cam_frames:
            if len(cam_frames[k]) > 0:
                cam_frames[k]["class"] = cam_frames[k]["class"].apply(
                    lambda x: obj_class[x])


def get_save_path(data_folder, ext):
    save_path = data_folder.copy()
    save_path[-1] = save_path[-1] + ext
    return save_path


def get_folders(folder_name, inlc_names, notincl_names):
    folders = os.listdir(folder_name)
    for name in notincl_names:
        folders = [f for f in folders if f.find(name) < 0]
    for name in inlc_names:
        folders = [f for f in folders if f.find(name) >= 0]
    return folders


def concat_all_frames(data):
    data_all = {}
    for d in data:
        data_all[d] = pd.concat(data[d])
    return data_all


# %% main
def get_cols_by_id(df: pd.DataFrame,
                   cols: list,
                   obj_id: str,
                   col: str = "id") -> list:
    return df[df[col].astype(str) == obj_id][cols].values.tolist()[0]


def transform_obj_bbox(obj: list):
    """
    get obj bbox in form of [minx, miny, maxx, maxy, orientation]
    from [pos_x, pos_y, length, width, heading]
    """
    x = obj[0]
    y = obj[1]
    half_len = obj[2] / 2
    half_w = obj[3] / 2
    heading = obj[4]
    return [x - half_len, y - half_w, x + half_len, y + half_w, heading]


def get_obj_bbox(obj_id: str,
                 df: pd.DataFrame,
                 col: str = "id",
                 cols: list = ["pos_x",
                               "pos_y",
                               "length",
                               "width",
                               "heading"]):
    """
    get obj bbox in form of [minx, miny, maxx, maxy, orientation]
    first get [pos_x, pos_y, length, width, heading] by get_cols_by_id
    """
    obj_pos_frame_hd = get_cols_by_id(df,
                                      cols,
                                      obj_id,
                                      col)
    # print(obj_id+'bbox')
    # print(transform_obj_bbox(obj_pos_frame_hd))
    return transform_obj_bbox(obj_pos_frame_hd)


def is_nan(value):
    return value != value


def get_iou(b1, b2):
    """
    get iou of 2 bbox
    input example:
    b1 = [35, 40, 65, 60, 0]
    b2 = [30, 30, 65, 70, 90]
    [minx, miny, maxx, maxy, orientation]
    orientation: radian of box rotation (anticlockwise is positive)
    affinity.rotate uses angle to calculate box -> need to change orientation
    to angular value
    """
    box1 = box(b1[0], b1[1], b1[2], b1[3])
    box2 = box(b2[0], b2[1], b2[2], b2[3])
    # check if orientation is nan value, if yes, change to 0 degree
    if is_nan(b1[4]):
        b1[4] = 0
    if is_nan(b2[4]):
        b2[4] = 0
    box1 = affinity.rotate(box1, b1[4] / np.pi * 180)
    box2 = affinity.rotate(box2, b2[4] / np.pi * 180)
    # iou = box1.intersection(box2).area / box1.union(box2).area
    # TODO: 用相交区域除以面积小的obj可能更好
    iou = box1.intersection(box2).area / (min(box1.area, box2.area))
    return iou


def get_dist_matrix(a: np.array,
                    b: np.array):
    # dm = distance_matrix(a, b)
    return pd.DataFrame(distance_matrix(a, b))

def get_dist_matrix_np(a: np.array,
                    b: np.array):
    return distance_matrix(a, b)


def get_nearest_objs(objs, objs2, df, num: int = 1):
    """
    get num nearest objects in objs2 for each object in objs based on parewise
    distmatrix (df)

    """
    return {obj: df[obj][objs2].sort_values()[0:num].index.to_list()
            for obj in objs}


def get_nearest_objs_np(list_row, list_col, list_r, list_c, array):
    """
    get num nearest objects in objs2 for each object in objs based on parewise
    distmatrix (df)

    """
    col_indices = [list_col.index(c) for c in list_c]
    row_indices = [list_row.index(d) for d in list_r]
    
    result = {}

    for i, c in enumerate(list_c):
        col = col_indices[i]
        # 提取相关的列数据
        column_data = [array[row][col] for row in row_indices]
        # 找到最小值的索引
        min_index = column_data.index(min(column_data))
        # 将结果添加到字典中
        result[c] = [list_r[min_index]]
    
    return result


def hysteresis(x, last_state):
    threshold_low = 0.5
    threshold_high = 0.52
    
    state = True
    
    if x > threshold_high:
        state = False
    elif x < threshold_low:
        state = True
    else:
        state = last_state

    return state


def update_dict_in_dict_value(data: dict, value_dict: dict):
    """
    input example:
    {'01': {'spd': 2, 'angle': 4, 'position': 4, 'status': 'mea'},
     '02': {'spd': 5, 'angle': 7, 'position': 9, 'status': 'pred'},
     '03': {'spd': 77, 'angle': 23, 'position': 15, 'status': 'new'}}
    update_dict_in_dict_value(input, {"status":"pred"})
    output example:
    {'01': {'spd': 2, 'angle': 4, 'position': 4, 'status': 'pred'},
     '02': {'spd': 5, 'angle': 7, 'position': 9, 'status': 'pred'},
     '03': {'spd': 77, 'angle': 23, 'position': 15, 'status': 'pred'}}
    """
    for k in data:
        data[k].update(value_dict)


def get_attr_from_dict_in_dict(data: dict, attr: str, out_type: str):
    """
    input example:
    {'01': {'spd': 2, 'angle': 4, 'position': 4, 'status': 'predicted'},
     '02': {'spd': 5, 'angle': 7, 'position': 9, 'status': 'predicted'},
     '03': {'spd': 77, 'angle': 23, 'position': 15, 'status': 'predicted'}}
    out = get_attr_from_dict_in_dict(input, "spd")
    output example:
    dict type: {'01':2, '02':5, '03':77}
    list type: [2, 5, 77]
    """
    if out_type == "dict":
        out = {k: data[k][attr] for k in data}
    elif out_type == "list":
        out = [data[k][attr] for k in data]
    return out


def update_dict_in_dict_value_for_key(data: dict,
                                      value_dict: dict,
                                      keys: list):
    """
    input example:
    {'01': {'spd': 2, 'angle': 4, 'position': 4, 'status': 'mea'},
     '02': {'spd': 5, 'angle': 7, 'position': 9, 'status': 'pred'},
     '03': {'spd': 77, 'angle': 23, 'position': 15, 'status': 'new'}}
    update_dict_in_dict_value(input, {"status":"pred"}, ["01"])
    output example:
    {'01': {'spd': 2, 'angle': 4, 'position': 4, 'status': 'pred'},
     '02': {'spd': 5, 'angle': 7, 'position': 9, 'status': 'pred'},
     '03': {'spd': 77, 'angle': 23, 'position': 15, 'status': 'new'}}
    """
    for k in keys:
        data[k].update(value_dict)


class CamPostProc():
    def __init__(self):
        # self.df = pd.DataFrame()
        # self.df_last = pd.DataFrame()
        self.hist_cols = ["id", "class", "status", "class_orig_cam"]
        # self.df_pp = pd.DataFrame(columns=self.hist_cols)
        self.df_cols = ["id", "class", "status", "pos_x", "pos_y",
                        "class_confidence", "width", "length", "heading",
                        "flag", "rel_spd", "rel_spd_lon", "rel_spd_lat"]
        self.t_cur = 0
        self.t_last = 0
        # self.hist = pd.DataFrame()
        self.paired_objs = {}
        self.filter_x_dict = {}
        self.filter_y_dict = {}
        self.confid_dict = {}
        self.state_x_dict = {}
        self.state_y_dict = {}
        self.dt = 0
        self.size_dict = {'pedestrian': (0.53, 0.5),
                          'bicycle': (1.89, 0.5),
                          'car': (4.075, 1.668),
                          'motorbike': (2.025, 0.67),
                          'bus': (9.6, 2.5),
                          'truck': (7.2, 2.3)}
        self.veh_spd_x = 0.
        self.veh_spd_y = 0.
        
        self.motion_dict = {}
        self.motion_confidence_dict = {}
        # code improvement
        self.d_last = {}
        self.d_pp = {}
        self.d = {}
        self.h_data = deque(maxlen=16)
        self.h_time = deque(maxlen=16)

    def _create_kf_filter_in_df(self, df_new):
        """
        @author: Yantong
        create a new KMFilter instance for the objects with status "new"
        """
        df_new["filter_x"] = df_new["pos_x"].map(lambda x: KMFilter(x))
        df_new["filter_y"] = df_new["pos_y"].map(lambda x: KMFilter(x))

    def _map_kf(self, df_to_map):
        """
        @author: Yantong
        use the filter_x_dict to map the filter_x with the corresponding id
        """
        df_to_map["filter_x"] = df_to_map["id"].map(self.filter_x_dict)
        df_to_map["filter_y"] = df_to_map["id"].map(self.filter_y_dict)

    def _update_kf_container(self):
        """
        @author: Yantong
        updates the filter_x/y_dict dictionary from the id and KMFilter
        instance of df_pp
        """
        self.filter_x_dict = dict(zip(self.df_pp["id"],
                                      self.df_pp["filter_x"]))
        self.filter_y_dict = dict(zip(self.df_pp["id"],
                                      self.df_pp["filter_y"]))
    def _update_kf_filter_dict(self):
        if len(self.d_pp) > 0:
            self.filter_x_dict.update(self.filter_x_dict_new)
            self.filter_y_dict.update(self.filter_y_dict_new)
            # ids = set(self.df_pp["id"])
            ids = set(list(self.d_pp.keys()))
            self.filter_x_dict = {key: value for key, value in self.filter_x_dict.items() if key in ids}
            self.filter_y_dict = {key: value for key, value in self.filter_y_dict.items() if key in ids}
        
        
    def _create_kf_filter_in_dict(self):
        """
        @author: Yantong
        create a new KMFilter instance for the objects with status "new"
        """
        self.filter_x_dict = {}
        self.filter_y_dict = {}
        ####################################################
        # code improvement, 害怕影响所以写在前面
        self.pos_x_dict = get_attr_from_dict_in_dict(self.d_pp, "pos_x", "dict")
        self.pos_y_dict = get_attr_from_dict_in_dict(self.d_pp, "pos_y", "dict")
        # print ("_create_kf_filter_in_dict, impr: ", self.pos_x_dict, self.pos_y_dict)
        ####################################################
        # self.pos_x_dict = dict(zip(self.df_pp["id"], self.df_pp["pos_x"]))
        # self.pos_y_dict = dict(zip(self.df_pp["id"], self.df_pp["pos_y"]))
        # print ("_create_kf_filter_in_dict, orig: ", self.pos_x_dict, self.pos_y_dict)
        for key, value in self.pos_x_dict.items():
            self.filter_x_dict[key] = KMFilter(value)
            
        for key, value in self.pos_y_dict.items():
            self.filter_y_dict[key] = KMFilter(value)

    def _create_kf_filter_new_in_dict(self, d_new):
        """
        @author: Yantong
        create a new KMFilter instance for the objects with status "new"
        """
        self.filter_x_dict_new = {}
        self.filter_y_dict_new = {}
        # pos_x_dict = dict(zip(df_new["id"], df_new["pos_x"]))
        # pos_y_dict = dict(zip(df_new["id"], df_new["pos_y"]))
        ####################################################
        # code improvement
        # print ("_create_kf_filter_new_in_dict orig: ", pos_x_dict, pos_y_dict)
        pos_x_dict = get_attr_from_dict_in_dict(d_new, "pos_x", "dict")
        pos_y_dict = get_attr_from_dict_in_dict(d_new, "pos_y", "dict")
        # print ("_create_kf_filter_new_in_dict impr: ", pos_x_dict, pos_y_dict)
        for key, value in pos_x_dict.items():
            self.filter_x_dict_new[key] = KMFilter(value)
            
        for key, value in pos_y_dict.items():
            self.filter_y_dict_new[key] = KMFilter(value)   

    def _create_motion_status_confid_in_dict(self): # not in use
        """
        @author: Yantong
        create a new KMFilter instance for the objects with status "new"
        """
        ####################################################
        # code improvement, 怕影响所以写咋前面
        update_dict_in_dict_value(self.d_pp, {"motion": "",
                                              "motion_confidence": 0.03})
        self.motion_confidence_dict = get_attr_from_dict_in_dict(
            self.d_pp, "motion_confidence", "dict")
        # print ("_create_motion_status_confid_in_dict impr: ", self.motion_confidence_dict)
        ####################################################
        # self.df_pp["motion"] = ""
        # self.df_pp["motion_confidence"] = 0.03
        # self.motion_confidence_dict = dict(zip(self.df_pp["id"],
        #                                        self.df_pp["motion_confidence"]))
        # print ("_create_motion_status_confid_in_dict orig: ", self.motion_confidence_dict)
        self.state_x_dict = {}
        self.state_y_dict = {}
        for key in self.motion_confidence_dict.keys():
            self.state_x_dict[key] = False
            self.state_y_dict[key] = False

    def _create_motion_status_confid_new_in_dict(self, d_new):
        """
        @author: Yantong
        create a new KMFilter instance for the objects with status "new"
        """
        # df_new["motion"] = ""
        # df_new["motion_confidence"] = 0.03
        # self.motion_confidence_dict_new = dict(zip(df_new["id"],
        #                                            df_new["motion_confidence"]))
        ####################################################
        # code improvement
        # print ("_create_motion_status_confid_new_in_dict orig: ", self.motion_confidence_dict_new)
        update_dict_in_dict_value(d_new, {"motion": "",
                                          "motion_confidence": 0.03})
        self.motion_confidence_dict_new = get_attr_from_dict_in_dict(d_new, "motion_confidence", "dict")
        # print ("_create_motion_status_confid_new_in_dict impr: ", self.motion_confidence_dict_new1)
        ####################################################
        self.state_x_dict_new = {}
        self.state_y_dict_new = {}
        for key in self.motion_confidence_dict_new.keys():
            self.state_x_dict_new[key] = False
            self.state_y_dict_new[key] = False
            
    def _update_state_xy_dict(self):
        
        # if len(self.df_pp) > 0:
        if len(self.d_pp) >0: ####################################################
            self.state_x_dict.update(self.state_x_dict_new)
            self.state_y_dict.update(self.state_y_dict_new)
            # 0718 new add
            self.motion_confidence_dict.update(self.motion_confidence_dict_new)
    
            # ids = set(self.df_pp["id"])
            ids = set(self.d_pp.keys()) ####################################################
            self.state_x_dict = {key: value for key, value in self.state_x_dict.items() if key in ids}
            self.state_y_dict = {key: value for key, value in self.state_y_dict.items() if key in ids}
            # 0718 new add
            self.motion_confidence_dict = {key: value for key, value in self.motion_confidence_dict.items() if key in ids}
            

    def _create_motion_status_confid_in_df(self, df_new):
        """
        @author: Yantong
        create a new KMFilter instance for the objects with status "new"
        """
        df_new["motion_confidence"] = 0.03
        df_new["rel_spd_lon_last_state"] = False
        df_new["rel_spd_lat_last_state"] = False

    def _map_confid(self, df_to_map):
        """
        @author: Yantong
        use the filter_x_dict to map the filter_x with the corresponding id
        """
        df_to_map["motion_confidence"] = df_to_map["id"].map(self.confid_dict)
        df_to_map["rel_spd_lon_last_state"] = df_to_map["id"].map(self.state_x_dict)
        df_to_map["rel_spd_lat_last_state"] = df_to_map["id"].map(self.state_y_dict)
        

    def _update_confid_container(self):
        """
        @author: Yantong
        updates the filter_x/y_dict dictionary from the id and KMFilter
        instance of df_pp
        """
        self.confid_dict = dict(zip(self.df_pp["id"],
                                      self.df_pp["motion_confidence"]))
        self.state_x_dict = dict(zip(self.df_pp["id"],
                                      self.df_pp["rel_spd_lon_last_state"]))
        self.state_y_dict = dict(zip(self.df_pp["id"],
                                      self.df_pp["rel_spd_lat_last_state"]))


    def update_input(self,
                     d_cam: dict,
                     t: float,
                     veh_spd: float,
                     steer_angle: float,
                     t_store_hist: float = 1.0):
        t1 = time.time()
        self.t_last = self.t_cur
        self.t_cur = float(t)
        self.dt = self.t_cur - self.t_last
        # self.df_last = self.df_pp.copy()
        # self.df = d_cam.copy()
        # if len(self.df) > 0:
        #     self.df["id"] = self.df["id"].astype("str")
        # else:
        #     self.df = pd.DataFrame(columns = self.df_cols)
        ####################################################
        # code improvement
        self.d_last = {k: v.copy() for k, v in self.d_pp.items()}
        self.d = {k: v.copy() for k, v in d_cam.items()}
        self.h_data.append(self.d_pp)
        self.h_time.append(self.t_last)
        ####################################################
        self._rename_retracked_obj()
        # temp_df = self.df_pp[self.hist_cols]
        # temp_df["t"] = self.t_last
        # self.hist = pd.concat([self.hist, temp_df], axis=0)
        # self.hist = self.hist[self.hist["t"] >= self.t_cur - t_store_hist]
        tyre_angle = get_tyre_angle(steer_angle)
        self.veh_spd_x = veh_spd * math.cos(tyre_angle) / 3.6
        self.veh_spd_y = veh_spd * math.sin(tyre_angle) / 3.6
        t2 = time.time()
        # print ("cam_pp_update_time: ", (t2-t1)*1000)

    def _rename_retracked_obj(self):
        # cur_objs = self.df["id"].to_list()
        # paired_objs_list = [[k, self.paired_objs[k]] for k in self.paired_objs]
        # for l in paired_objs_list:
        #     overlap = list(set(cur_objs) & set(l))
        #     if len(overlap) == 2:
        #         self.paired_objs.pop(l[0])
        # self._check_paired_objs()
        # TODO here self._check_paired_objs() directly change paired_objs,
        # then rename the ids of df using pared_objs. 
        # a better way is: do not change paired_objs, but establish a new dict
        # based on paired_objs according to ids of df, then rename ids
        # in this frame using the new dict. This can avoid deleting useful
        # paired info
        # self.df["id"] = self.df["id"].apply(lambda x: self.paired_objs[x]
        #                           if x in self.paired_objs.keys() else x)
        ####################################################
        # code improvement, checked same as before
        paired_id_list = list(self.paired_objs.keys())
        retrack_id_list = []
        for k in self.d:
            if k in paired_id_list:
                retrack_id_list.append(k)
        if retrack_id_list:
            for k in retrack_id_list:
                self.d[self.paired_objs[k]] = self.d.pop(k)

    def _compare_frames(self):
        # objs_cur = self.df["id"].to_list()
        # objs_last = self.df_last["id"].to_list()
        ####################################################
        # code improvement
        # print ("_compare_frames orig: ", objs_cur, objs_last)
        objs_cur = list(self.d.keys())
        objs_last = list(self.d_last.keys())
        # print ("_compare_frames impr: ", objs_cur, objs_last)
        self.objs_exist_both = list(set(objs_cur).intersection(set(objs_last)))
        self.objs_disappear = list(set(objs_last) - set(self.objs_exist_both))
        self.objs_new = list(set(objs_cur) - set(self.objs_exist_both))

    def _get_dfpp_of_measured_obj(self):
        # self.df_pp = self.df[self.df["id"].isin(self.objs_exist_both)].copy()
        # self.df_pp["status"] = "measured"
        ####################################################
        # code improvement
        self.d_pp = {k: self.d[k].copy() for k in self.d if k in self.objs_exist_both}
        update_dict_in_dict_value(self.d_pp, {"status": "measured"})
        # print ("_get_dfpp_of_measured_obj df_pp", df_pp)
        # print ("_get_dfpp_of_measured_obj d_pp", self.d_pp)
        ####################################################

    def _get_obj_tobe_extrap(self,
                             t_thrd: float = 1,
                             show_time_thrd: float = 12,
                             need_cols: list = ["t", "id", "status"]):
        """
        output disappeared objects that really existed for show_time_thrd
        times in past t_thrd seconds
        -> an extrapolation of these objects is needed
        """
        #===================================================================
        ##原始方法，基于dataframe
        # df = self.hist[self.hist["t"] >= self.t_cur - t_thrd][need_cols]
        # df = df[df["status"] != "predicted"]
        # df = df[df["id"].isin(self.objs_disappear)]
        # show_times = dict(df.groupby("id")["t"].agg("count"))
        #===================================================================
        # 新的方法使用list
        # 从self.hist中提取需要的列数据
        # t_list = self.hist["t"].tolist()
        # id_list = self.hist["id"].tolist()
        # status_list = self.hist["status"].tolist()
        # show_times = {}
        
        # for t, id, status in zip(t_list, id_list, status_list):
        #     if (t >= self.t_cur - t_thrd and 
        #         status != "predicted" and 
        #         id in self.objs_disappear):
        #         # 如果id已经在字典中，增加计数；否则，初始化为1
        #         if id in show_times:
        #             show_times[id] += 1
        #         else:
        #             show_times[id] = 1
        #=================================================================
        ####################################################
        # code improvement
        # print ("_get_obj_tobe_extrap orig: ", show_times)
        self.h_time_list = list(self.h_time)
        self.h_data_list = list(self.h_data)
        show_times_list = []
        show_times = {}
        for t, d in zip(self.h_time_list, self.h_data_list):
            if t >= self.t_cur - t_thrd:
                objs1 = {k: d[k] for k in d if
                          k in self.objs_disappear}
                objs2 = {k: objs1[k] for k in objs1 if
                          objs1[k]["status"] != "predicted"}
                show_times_list.extend(objs2.keys())
        for k in show_times_list:
            show_times[k] = show_times.get(k,0) + 1
        # print ("_get_obj_tobe_extrap impr: ", show_times)
        ####################################################
        self.obj_extrap = [obj for obj in show_times
                           if show_times[obj] >= show_time_thrd]

    def _get_dfpp_of_predicted_obj(self):
        """
        copy attributes of obj_extrap from last frame directly and set status
        as predicted. For predicted objects, and extrapolation of position and
        speed is needed
        """
        # self.df_extrap = (self.df_last[self.df_last["id"].isin(self.obj_extrap)].copy())
        # self.df_extrap["status"] = "predicted"
        # self.df_pp = pd.concat([self.df_pp, self.df_extrap], axis=0)
        # self._map_kf(self.df_pp)  # legacy
        # self._map_confid(self.df_pp)
        ####################################################
        # code improvement
        # print ("_get_dfpp_of_predicted_obj orig: ", self.df_pp)
        self.d_extrap = {k: self.d_last[k] for k in self.d_last if
                         k in self.obj_extrap}
        update_dict_in_dict_value(self.d_extrap, {"status": "predicted"})
        self.d_pp.update(self.d_extrap)
        # print ("_get_dfpp_of_predicted_obj impr: ", self.d_pp)
        

    def _get_dfpp_of_new_obj(self):
        """
        copy attributes of objs_new from current frame and set status as new
        """
        # self.df_new = (self.df[self.df["id"].isin(self.objs_new)].copy())
        # self.df_new["status"] = "new"
        # self._create_kf_filter_in_df(df_new)  # legacy
        ####################################################
        # code improvement
        self.d_new = {k: self.d[k].copy() for k in self.d if k in self.objs_new}
        update_dict_in_dict_value(self.d_new, {"status": "new"})
        ####################################################
        self._create_kf_filter_new_in_dict(self.d_new)
        self._create_motion_status_confid_new_in_dict(self.d_new)
        # self._create_motion_status_confid_in_df(df_new)  # legacy
        # self._create_df_filter(df_new)
        # self.df_pp = pd.concat([self.df_pp, self.df_new], axis=0)
        ####################################################
        # code improvement
        self.d_pp.update(self.d_new)
        ####################################################
        
    # def _filter_and_replace(self, 
    #                         filt_target: str,
    #                         meas_data: str,
    #                         filt_type: str):
    #     self.df_pp[filt_target] = self.df_pp.apply(
    #         lambda row: row[filt_type].km_filter(self.dt, row[meas_data])[0, 0]
    #         if row["status"] == "measured"
    #         else row[filt_type].km_extrapolate(self.dt)[0, 0]
    #         if row["status"] == "predicted"
    #         else row[filt_target],
    #         axis=1)
    



    def _update_pos_spd_dict(self, mtd: str = "mm"):
        """
        @author: Yantong
        for measured objects:
        filter pos_x, pos_y, rel_spd_lon, rel_spd_lat
        for predicted objects:
        extrapolate pos_x, pos_y, rel_spd_lon, rel_spd_lat
        for new objects:
        keeps the origninal value
        """
        # The following are used when the 'new' obejcts does not need to filter
        # at the first step.
        # if len(self.df_pp) > 0:
        #     self.status_dict = dict(zip(self.df_pp["id"], self.df_pp["status"]))
        #     self.pos_x_dict = dict(zip(self.df_pp["id"], self.df_pp["pos_x"]))
        #     self.pos_y_dict = dict(zip(self.df_pp["id"], self.df_pp["pos_y"]))
        #     self.rel_spd_lon_dict = dict(zip(self.df_pp["id"], self.df_pp["rel_spd_lon"]))
        #     self.rel_spd_lat_dict = dict(zip(self.df_pp["id"], self.df_pp["rel_spd_lat"]))
        #     for key, value in self.pos_x_dict.items():
        #         if self.status_dict[key] == "measured":
        #             pos_x, rel_spd_lon = self.filter_x_dict[key].km_filter(self.dt, self.pos_x_dict[key])
        #             pos_y, rel_spd_lat = self.filter_y_dict[key].km_filter(self.dt, self.pos_y_dict[key])
        #         elif self.status_dict[key] == "predicted":
        #             pos_x, rel_spd_lon = self.filter_x_dict[key].km_extrapolate(self.dt)
        #             pos_y, rel_spd_lat = self.filter_y_dict[key].km_extrapolate(self.dt)
        #         else:
        #             # pos_x, rel_spd_lon = self.pos_x_dict[key], self.rel_spd_lon_dict[key]
        #             # pos_y, rel_spd_lat = self.pos_y_dict[key], self.rel_spd_lat_dict[key]
        #             pos_x, rel_spd_lon = self.pos_x_dict[key], 0
        #             pos_y, rel_spd_lat = self.pos_y_dict[key], 0

        #         self.pos_x_dict[key] = pos_x
        #         self.pos_y_dict[key] = pos_y
        #         self.rel_spd_lon_dict[key] = rel_spd_lon
        #         self.rel_spd_lat_dict[key] = rel_spd_lat

        #     self.id_list = self.df_pp["id"].values
        #     self.df_pp["pos_x"] = [self.pos_x_dict[i] for i in self.id_list]
        #     self.df_pp["pos_y"] = [self.pos_y_dict[i] for i in self.id_list]
        #     self.df_pp["rel_spd_lon"] = [self.rel_spd_lon_dict[i] for i in self.id_list]
        #     self.df_pp["rel_spd_lat"] = [self.rel_spd_lat_dict[i] for i in self.id_list]
            # ==========================================================================================
            # 一种提升时间的旧方法
            # for key, value in self.pos_x_dict.items():
            #     if self.status_dict[key] == "measured":
            #         pos_x, rel_spd_lon = self.filter_x_dict[key].km_filter(self.dt, self.pos_x_dict[key])
            #         pos_y, rel_spd_lat = self.filter_y_dict[key].km_filter(self.dt, self.pos_y_dict[key])
            #     elif self.status_dict[key] == "predicted":
            #         pos_x, rel_spd_lon = self.filter_x_dict[key].km_extrapolate(self.dt)
            #         pos_y, rel_spd_lat = self.filter_y_dict[key].km_extrapolate(self.dt)
            #     else:
            #         continue
            #     self.df_pp.loc[self.df_pp["id"] == key, "pos_x"] = pos_x
            #     self.df_pp.loc[self.df_pp["id"] == key, "pos_y"] = pos_y
            #     self.df_pp.loc[self.df_pp["id"] == key, "rel_spd_lon"] = rel_spd_lon
            #     self.df_pp.loc[self.df_pp["id"] == key, "rel_spd_lat"] = rel_spd_lat
            
            # self.df_pp["pos_x"] = self.df_pp["id"].map(self.pos_x_dict)
            # self.df_pp["pos_y"] = self.df_pp["id"].map(self.pos_y_dict)
            # self.df_pp["rel_spd_lon"] = self.df_pp["id"].map(self.rel_spd_lon_dict)
            # self.df_pp["rel_spd_lat"] = self.df_pp["id"].map(self.rel_spd_lat_dict)
        ####################################################
        # code improvement
        # print ("_update_pos_spd_dict orig: ", self.df_pp[["id", "pos_x", "pos_y", "rel_spd_lon", "rel_spd_lat"]])
        if len(self.d_pp) > 0:
            for k, v in self.d_pp.items():
                if v["status"] == "measured":
                    pos_x, rel_spd_lon = self.filter_x_dict[k].km_filter(self.dt, self.d_pp[k]["pos_x"])
                    pos_y, rel_spd_lat = self.filter_y_dict[k].km_filter(self.dt, self.d_pp[k]["pos_y"])
                elif v["status"] == "predicted":
                    pos_x, rel_spd_lon = self.filter_x_dict[k].km_extrapolate(self.dt)
                    pos_y, rel_spd_lat = self.filter_y_dict[k].km_extrapolate(self.dt)
                else:
                    pos_x, rel_spd_lon = self.d_pp[k]["pos_x"], 0.0
                    pos_y, rel_spd_lat = self.d_pp[k]["pos_y"], 0.0
                update_dict = {"pos_x": pos_x,
                               "pos_y": pos_y,
                               "rel_spd_lon": rel_spd_lon,
                               "rel_spd_lat": rel_spd_lat}
                # print ("self.d_pp before update: ", {k: {self.d_pp[k][k2] for k2 in ["pos_x", "pos_y"]}})
                update_dict_in_dict_value_for_key(self.d_pp, update_dict, [k])
                # print ("update_dict: ", update_dict)
                # print ("self.d_pp: ", {k: {self.d_pp[k][k2] for k2 in ["pos_x", "pos_y"]}})
        # print ("_update_pos_spd_dict impr: ", self.d_pp)
        ####################################################


    def _update_pos_spd(self, mtd: str = "mm"):
        """
        @author: Yantong
        for measured objects:
        filter pos_x, pos_y, rel_spd_lon, rel_spd_lat
        for predicted objects:
        extrapolate pos_x, pos_y, rel_spd_lon, rel_spd_lat
        for new objects:
        keeps the origninal value
        """
        #=====================================================================
        # self._filter_and_replace("pos_x", "pos_x", "filter_x")
        # self._filter_and_replace("rel_spd_lon", "pos_x", "filter_x")
        # self._filter_and_replace("pos_y", "pos_y", "filter_y")
        # self._filter_and_replace("rel_spd_lat", "pos_y", "filter_y")

        #======================================================================
        # self.df_pp["pos_x"] = self.df_pp.apply(
        #     lambda row: row["filter_x"].km_filter(self.dt, row["pos_x"])[0]
        #     if row["status"] == "measured"
        #     else row["filter_x"].km_extrapolate(self.dt)[0]
        #     if row["status"] == "predicted"
        #     else row["pos_x"],
        #     axis=1)
        # if len(self.df_pp) > 0:
        #     self.df_pp[["pos_x", "rel_spd_lon"]] = self.df_pp.apply(
        #         lambda row: row["filter_x"].km_filter(self.dt, row["pos_x"])
        #         if row["status"] == "measured"
        #         else row["filter_x"].km_extrapolate(self.dt)
        #         if row["status"] == "predicted"
        #         else row["filter_x"].km_filter(0, row["pos_x"]),
        #         axis=1,
        #         result_type='expand')

        #     self.df_pp[["pos_y", "rel_spd_lat"]] = self.df_pp.apply(
        #         lambda row: row["filter_y"].km_filter(self.dt, row["pos_y"])
        #         if row["status"] == "measured"
        #         else row["filter_y"].km_extrapolate(self.dt)
        #         if row["status"] == "predicted"
        #         else row["filter_y"].km_filter(0, row["pos_x"]),
        #         axis=1,
        #         result_type='expand')
        
        # The following are used when the 'new' obejcts does not need to filter
        # at the first step.
        if len(self.df_pp) > 0:
            self.df_pp[["pos_x", "rel_spd_lon"]] = self.df_pp.apply(
                lambda row: row["filter_x"].km_filter(self.dt, row["pos_x"])
                if row["status"] == "measured"
                else row["filter_x"].km_extrapolate(self.dt)
                if row["status"] == "predicted"
                else row[["pos_x", "rel_spd_lon"]],
                axis=1,
                result_type='expand')

            self.df_pp[["pos_y", "rel_spd_lat"]] = self.df_pp.apply(
                lambda row: row["filter_y"].km_filter(self.dt, row["pos_y"])
                if row["status"] == "measured"
                else row["filter_y"].km_extrapolate(self.dt)
                if row["status"] == "predicted"
                else row[["pos_y", "rel_spd_lat"]],
                axis=1,
                result_type='expand')

        # TODO comment Jing
        # self.df_pp["pos_x"] = self.df_pp.apply(
        #     lambda x:
        #         filter_x_dict[x["id"]].km_filter(self.dt, x["pos_x"])[0, 0] if...)

    def _filter_and_extrapolation(self):
        self._compare_frames()
        self._get_dfpp_of_measured_obj()
        self._get_obj_tobe_extrap()
        self._get_dfpp_of_predicted_obj()
        self._get_dfpp_of_new_obj()
        # self._update_pos_spd()  # legacy
        self._update_kf_filter_dict()
        self._update_pos_spd_dict()


    def _get_pairwise_dist(self, objs, cols: list = ["pos_x",
                                                     "pos_y"]): # not in use
#=================================================================================
        pos_x_y_new = self.df_pp[self.df_pp["id"].isin(self.objs_new)][cols]
        pos_x_y_new_index = self.df_pp[self.df_pp["id"].isin(self.objs_new)]['id']

        pos_x_y = self.df_pp[self.df_pp["id"].isin(objs)][cols]
        pos_x_y_index = self.df_pp[self.df_pp["id"].isin(objs)]['id']

        self.pair_dist = get_dist_matrix(pos_x_y_new, pos_x_y)
        self.pair_dist.index = pos_x_y_new_index
        self.pair_dist.columns = pos_x_y_index
        

    def _get_pairwise_dist_np(self, objs, cols: list = ["pos_x",
                                                     "pos_y"]):        
        # pos_x_new_list = self.df_new["pos_x"].tolist()
        # pos_y_new_list = self.df_new["pos_y"].tolist()
        # pos_x_y_new_np = np.array(list(zip(pos_x_new_list, pos_y_new_list)))
        # self.pos_x_y_new_index_list = self.df_new["id"].tolist()
        
        # pos_x_list = self.df_pp["pos_x"].tolist()
        # pos_y_list = self.df_pp["pos_y"].tolist()
        # pos_x_y_np = np.array(list(zip(pos_x_list, pos_y_list)))
        # self.pos_x_y_index_list = self.df_pp["id"].tolist()     

        # self.pair_dist_np = get_dist_matrix_np(pos_x_y_new_np, pos_x_y_np)
        ####################################################
        # code improvement
        # print("_get_pairwise_dist_np orig: ", self.pos_x_y_new_index_list, self.pos_x_y_index_list, self.pair_dist_np)
        pos_x_new_list = get_attr_from_dict_in_dict(self.d_new, "pos_x", "list")
        pos_y_new_list = get_attr_from_dict_in_dict(self.d_new, "pos_y", "list")
        pos_x_y_new_np = np.array(list(zip(pos_x_new_list, pos_y_new_list)))
        self.pos_x_y_new_index_list = list(self.d_new.keys())
        pos_x_list = get_attr_from_dict_in_dict(self.d_pp, "pos_x", "list")
        pos_y_list = get_attr_from_dict_in_dict(self.d_pp, "pos_y", "list")
        pos_x_y_np = np.array(list(zip(pos_x_list, pos_y_list)))
        self.pos_x_y_index_list = list(self.d_pp.keys())
        self.pair_dist_np = get_dist_matrix_np(pos_x_y_new_np, pos_x_y_np)
        # print("_get_pairwise_dist_np impr: ", self.pos_x_y_new_index_list, self.pos_x_y_index_list, self.pair_dist_np)
        ####################################################

    def _get_pairwise_iou(self, objs):
        """
        get iou of nearest obj_extrap and objs_new, form of nearest_ious:
            {extrap_id1: {new_id1: iou, new_id2: iou},
             extrap_id2: {new_id1: iou, new_id3: iou},
             ...}
        example of self.nearest_ious:
        {"7": {"6": 0.9, "4": 0.8, "2": 0.89},
         "5": {"6": 0.85, "4": 0.3, "2": 0.86, "1": 0.1}}
        """
#=================================================================================
# 原始方法
        # self._get_pairwise_dist(objs)
        # nearest_objs = get_nearest_objs(self.obj_extrap + self.objs_exist_both,
        #                                 self.objs_new,
        #                                 self.pair_dist)

        # print('!!!!!')
        # print(nearest_objs)
        # # self.h = self.df_pp
        # self.nearest_ious = {obj: {obj2: get_iou(get_obj_bbox(obj, self.df_pp),
        #                                           get_obj_bbox(obj2, self.df_pp)
        #                                           )
        #                             for obj2 in nearest_objs[obj]}
        #                       for obj in nearest_objs}
#         print()
#=================================================================================
#新方法_list+np

        self._get_pairwise_dist_np(objs)
        ####################################################
        # code improvement
        nearest_objs = get_nearest_objs_np(self.pos_x_y_new_index_list,
                                            self.pos_x_y_index_list,
                                            self.objs_new,
                                            self.obj_extrap + self.objs_exist_both,
                                            self.pair_dist_np)
        ####################################################
        ####################################################
        # code improvement担心影响后面所以改在前面
        id_list = list(self.d_pp.keys()) # calculated before
        pos_x_list = get_attr_from_dict_in_dict(self.d_pp, "pos_x", "list")
        pos_y_list = get_attr_from_dict_in_dict(self.d_pp, "pos_y", "list")
        length_list = get_attr_from_dict_in_dict(self.d_pp, "length", "list")
        width_list = get_attr_from_dict_in_dict(self.d_pp, "width", "list")
        heading_list = get_attr_from_dict_in_dict(self.d_pp, "heading", "list")
        # print ("_get_pairwise_iou impr: ", id_list, pos_x_list, pos_y_list, length_list, width_list, heading_list)
        ####################################################
        # id_list = self.df_pp["id"].tolist()
        # pos_x_list = self.df_pp["pos_x"].tolist()
        # pos_y_list = self.df_pp["pos_y"].tolist()
        # length_list = self.df_pp["length"].tolist()
        # width_list = self.df_pp["width"].tolist()
        # heading_list = self.df_pp["heading"].tolist()
        # print ("_get_pairwise_iou orig: ", id_list, pos_x_list, pos_y_list, length_list, width_list, heading_list)
        self.nearest_ious = {obj: {obj2: get_iou(self.get_obj_bbox_list(obj,
                                                                        id_list,
                                                                        pos_x_list,
                                                                        pos_y_list,
                                                                        length_list,
                                                                        width_list,
                                                                        heading_list),
                                                  self.get_obj_bbox_list(obj2,
                                                                        id_list,
                                                                        pos_x_list,
                                                                        pos_y_list,
                                                                        length_list,
                                                                        width_list,
                                                                        heading_list)
                                                  )
                                    for obj2 in nearest_objs[obj]}
                              for obj in nearest_objs}

    def get_obj_bbox_list(self,
                          obj_id: str,
                          id_list: list,
                          pos_x_list: list,
                          pos_y_list: list,
                          length_list: list,
                          width_list: list,
                          heading_list: list):
        index = id_list.index(obj_id)
        obj_pos_frame_hd = [pos_x_list[index],
                            pos_y_list[index],
                            length_list[index],
                            width_list[index],
                            heading_list[index]]

        return transform_obj_bbox(obj_pos_frame_hd)
#=================================================================================

    def _check_paired_objs(self):
        """
        input examp. of paired_objs: {'617': '564', '639': '669', '771': '669'}
        output example:{'617': '564', '771': '669'}
        """
        l_temp = [self.paired_objs[k] for k in self.paired_objs]
        c = list(set([n for n in l_temp if l_temp.count(n) > 1]))
        id_tobe_removed = []
        for value in c:
            b = [k for k in self.paired_objs if self.paired_objs[k] == value]
            max_id = str(max([int(x) for x in b]))
            b.remove(max_id)
            id_tobe_removed.extend(b)
        for obj in id_tobe_removed:
            self.paired_objs.pop(obj)

    def _pair_ghost_with_new_objs(self, df_ghost):
        """
        pair extrapolated objects with new objects if they have overlap > set
        iou value. One new_id will be paired to one extrap_id, if more than one
        ious is above set value, the ids with highest iou will be paired
        Form of paired_objs:
            {new_id1: extrap_id1, new_id2: extrap_id2...}
        """
        df_rank_1 = df_ghost.rank(ascending=0, method="dense")
        df_rank_2 = df_ghost.rank(ascending=0, method="dense", axis=1)
        df_rank = df_rank_1 + df_rank_2
        num = max(df_rank.shape)
        self.paired_objs_cur_frame = {}
        for i in range(2, 2 + num):
            mask = df_rank == i
            paired = mask.apply(lambda x: list(x[x].index), axis=1).to_dict()
            paired = {new_obj: paired[new_obj][0] for new_obj in paired
                      if len(paired[new_obj]) > 0}
            df_rank = df_rank.drop(paired.keys(), axis=0)
            df_rank = df_rank.drop(paired.values(), axis=1)
            for k in paired:
                self.paired_objs[k] = paired[k]
                self.paired_objs_cur_frame[k] = paired[k]

    def _get_ghost_obj_by_iou(self, iou: float = 0.6):
        """
        pair extrapolated (predicted) objects with new objects by iou threshold
        """
        df_ghost = pd.DataFrame(self.nearest_ious)
        df_ghost = (df_ghost[df_ghost >= iou].dropna(
            axis=0, how="all").dropna(axis=1, how="all"))
        ####################################################
        # code improvement, 暂时先不improve了，目前因为没有pair时间消耗小
        # ghost = {k: self.nearest_ious[k] for k in self.nearest_ious if
        #          self.nearest_ious[k] >= iou}
        ####################################################
        # print('~~')
        # print(df_ghost)
        self._pair_ghost_with_new_objs(df_ghost)

    def _destroy_extrap_and_rename_new(self):
        # self.df_pp["orig_id_cam"] = self.df_pp["id"]
        
        # self.df_pp_pop = self.df_pp[self.df_pp["id"].isin(
        #   [self.paired_objs_cur_frame[k] for k in self.paired_objs_cur_frame])]

        # self.df_pp = self.df_pp[~self.df_pp["id"].isin(
        #   [self.paired_objs_cur_frame[k] for k in self.paired_objs_cur_frame])]
        # # TODO: rename using paired_objs_cur_frame
        # self.df_pp["id"] = self.df_pp["id"].apply(lambda x: self.paired_objs[x]
        #                           if x in self.paired_objs.keys() else x)

        # self.df_pp_pop.index = self.df_pp.loc[self.df_pp['id'].isin(
        #     self.df_pp_pop['id']), ["rel_spd_lon", "rel_spd_lat"]].index

        # self.df_pp.loc[self.df_pp['id'].isin(self.df_pp_pop['id']),
        #     ["rel_spd_lon", "rel_spd_lat"]] = self.df_pp_pop[
        #         ["rel_spd_lon", "rel_spd_lat"]]
        ####################################################
        # code improvement
        for k in self.d_pp:
            self.d_pp[k].update({"orig_id_cam": k})
        extrap_objs_list = [self.paired_objs_cur_frame[k] for k in
                            self.paired_objs_cur_frame]
        # find extrapolated obj
        self.d_pp_pop = {k: self.d_pp[k].copy() for k in self.d_pp if k in extrap_objs_list}
        # destroy extraplolated obj
        self.d_pp = {k: self.d_pp[k] for k in self.d_pp if k not in
                          extrap_objs_list}
        # rename new obj name with extrapolated obj name
        # and use extrapolated obj's speed for new obj
        for k in self.paired_objs_cur_frame:
            extrap_obj = self.paired_objs_cur_frame[k]
            update_dict = {"rel_spd_lon":
                           self.d_pp_pop[extrap_obj]["rel_spd_lon"],
                           "rel_spd_lat":
                               self.d_pp_pop[extrap_obj]["rel_spd_lat"]}
            update_dict_in_dict_value_for_key(self.d_pp, update_dict, [k])
            self.d_pp[extrap_obj] = self.d_pp.pop(k)
        # print ("_destroy_extrap_and_rename_new orig: ", self.df_pp, len(self.df_pp))
        # print ("_destroy_extrap_and_rename_new impr: ", self.d_pp.keys(), len(self.d_pp))
        ####################################################

    def _destroy_overlap_ghost_obj(self):
        """
        if an object with old id is predicted in this frame, but the real
        object is measured with a new id, they may have overlap and the old
        one is a ghost.
        This function checks the overlap of predicted object and new object
        in this frame and delete the ghost by checking iou and rename the new
        object with old id

        """
        objs = self.obj_extrap + self.objs_exist_both + self.objs_new
        if len(self.objs_new) > 0 and len(self.obj_extrap 
                                          + self.objs_exist_both) > 0:
            self._get_pairwise_iou(objs)
            self._get_ghost_obj_by_iou()
            self._destroy_extrap_and_rename_new()

    def _update_heading(self):
        """
        update heading with speed direction -> current no need
        """
        pass

    def _get_id_w_diff_class(self, cols: list = ["id", "class"]):
        """
        id_with_diff_class contains the ids with different class of this frame
        and last frame
        """
        #=====================================================================
        ## 原始方法
        # df1 = self.df_pp[cols]
        # df1.set_index("id", inplace=True)
        # df2 = self.df_last[cols]
        # df2.set_index("id", inplace=True)
        # df2 = df2.rename(columns={"class": "class_last"})
        # df_class = pd.concat([df1, df2], axis=1)
        # df_class = df_class.dropna(axis=0, how="any")
        # self.id_with_diff_class = (df_class[df_class["class"] != df_class["class_last"]].index.astype(str).to_list())
        #======================================================================
        # 从 self.df_pp 和 self.df_last 中提取需要的数据
        # df_pp_data = list(zip(self.df_pp["id"].tolist(), self.df_pp["class"].tolist()))
        # df_last_data = list(zip(self.df_last["id"].tolist(), self.df_last["class"].tolist()))
        ####################################################
        # code improvement, 害怕影响后面所以写在前面
        df1 = get_attr_from_dict_in_dict(self.d_pp, "class", "dict")
        df2 = get_attr_from_dict_in_dict(self.d_last, "class", "dict")
        # print ("_get_id_w_diff_class impr: ", df1, df2,len(df1), len(df1))
        ####################################################
        # 创建字典，用 id 作为键
        # df1 = {row[0]: row[1] for row in df_pp_data}
        # df2 = {row[0]: row[1] for row in df_last_data}
        # print ("_get_id_w_diff_class orig: ", df1, df2,len(df1), len(df1))
        # 找出在两个字典中都存在的 id，并比较它们的 class
        self.id_with_diff_class = []
        for objs_id in set(df1.keys()) & set(df2.keys()):
            if df1[objs_id] is not None and df2[objs_id] is not None:  # 相当于 dropna
                if df1[objs_id] != df2[objs_id]:
                    self.id_with_diff_class.append(str(objs_id))

    def _get_class(self, t_thrd: float = 1,
                   need_cols: list = ["t", "id", "class_orig_cam", "status"]):
        """
        get class with highest occurrence in last t_thrd seconds for ids with
        changed class
        Form of _obj_class:
            {id: class, id: class...}
        """
        # df = self.hist[self.hist["t"] >= self.t_cur - t_thrd][need_cols]
        # df = df[df["status"] != "predicted"]
        # df = df[df["id"].isin(self.id_with_diff_class)]
        # grouped = df.groupby(
        #     ["id", "class_orig_cam"])["class_orig_cam"].count().unstack()
        # self._obj_class = grouped.idxmax(axis=1).to_dict()
        ####################################################
        # code improvement
        # print("_get_class orig: ", self._obj_class, grouped)
        class_list = []
        id_list = []
        show_times = {}
        for t, d in zip(self.h_time_list, self.h_data_list):
            if t >= self.t_cur - t_thrd:
                objs1 = {k: d[k] for k in d if
                          k in self.id_with_diff_class}
                objs2 = {k: objs1[k] for k in objs1 if
                          objs1[k]["status"] != "predicted"}
                classes = get_attr_from_dict_in_dict(objs2, "class", "dict")
                class_list.append(classes)
                id_list.extend(objs2.keys())
        id_list = list(set(id_list))
        for k in id_list:
            show_times[k] = [x[k] for x in class_list if k in x]
        self._obj_class = {k: max(show_times[k], key=show_times[k].count)
                           for k in show_times}
        # print("_get_class impr: ", self._obj_class, show_times)
        ####################################################

    def _update_class(self):
        # self.df_pp["class"] = (
        #     self.df_pp.apply(lambda x: self._obj_class[x.id] if x.id in
        #                      self._obj_class.keys() else x["class"], axis=1))

        # self.df_pp[['length', "width"]] = (
        #     self.df_pp.apply(lambda x: self.size_dict[x["class"]]
        #                       if x["id"] in self._obj_class.keys()
        #                       else x[['length', "width"]],
        #                       axis=1,
        #                       result_type='expand'))
        ####################################################
        # code improvement
        # print("_update_class orig: ", self.df_pp[["id", "class"]])
        keys = list(self._obj_class.keys())
        for k in keys:
            update_dict = {"class": self._obj_class[k],
                           "length": self.size_dict[self._obj_class[k]][0],
                           "width": self.size_dict[self._obj_class[k]][1]}
            update_dict_in_dict_value_for_key(self.d_pp, update_dict, [k])
        # print("_update_class orig: ", get_attr_from_dict_in_dict(self.d_pp, "class", "dict"))
        ####################################################

    def _check_n_update_class(self, t_thrd: float = 1,
                              need_cols: list = ["t",
                                                 "id", "class", "status"]):
        """
        use class with highest occurrence for same id when observing
        class change
        """
        self._get_id_w_diff_class()
        # self.df_pp["class_orig_cam"] = self.df_pp["class"]
        ####################################################
        # code improvement, 害怕影响后面所以写在前面
        for k in self.d_pp:
            self.d_pp[k].update({"class_orig_cam": self.d_pp[k]["class"]})
        ####################################################
        if len(self.id_with_diff_class) > 0:
            self._get_class()
            self._update_class()
    
    def _get_agent_motion_status_dict(self, veh_spd_x, veh_spd_y):
        # if len(self.df_pp) > 0:
            ####################################################
            # code improvement, 害怕影响后面所以写在前面
        if len(self.d_pp) > 0:
            self.status_dict = get_attr_from_dict_in_dict(self.d_pp, "status", "dict") # used before, need to check later
            self.rel_spd_lon_dict = get_attr_from_dict_in_dict(self.d_pp, "rel_spd_lon", "dict") # used before, need to check later
            self.rel_spd_lat_dict = get_attr_from_dict_in_dict(self.d_pp, "rel_spd_lat", "dict") # used before, need to check later
            self.pos_x_dict = get_attr_from_dict_in_dict(self.d_pp, "pos_x", "dict") # used before, need to check later
            self.pos_y_dict = get_attr_from_dict_in_dict(self.d_pp, "pos_y", "dict") # used before, need to check later     
            # print ("_get_agent_motion_status_dict impr: ", self.status_dict,
            #        self.rel_spd_lon_dict, self.rel_spd_lat_dict,
            #        self.pos_x_dict, self.pos_y_dict)
            ####################################################
            # self.status_dict = dict(zip(self.df_pp["id"], self.df_pp["status"]))
            # self.rel_spd_lon_dict = dict(zip(self.df_pp["id"], self.df_pp["rel_spd_lon"]))
            # self.rel_spd_lat_dict = dict(zip(self.df_pp["id"], self.df_pp["rel_spd_lat"]))
            # self.pos_x_dict = dict(zip(self.df_pp["id"], self.df_pp["pos_x"]))
            # self.pos_y_dict = dict(zip(self.df_pp["id"], self.df_pp["pos_y"]))
            # print ("_get_agent_motion_status_dict orig: ", self.status_dict,
            #        self.rel_spd_lon_dict, self.rel_spd_lat_dict,
            #        self.pos_x_dict, self.pos_y_dict)
            for key in self.pos_x_dict.keys():
                if (hysteresis(abs(veh_spd_x + self.rel_spd_lon_dict[key]), self.state_x_dict[key])
                    and 
                    hysteresis(abs(veh_spd_y + self.rel_spd_lat_dict[key]), self.state_y_dict[key])):
                    self.motion_dict[key] = "stationary"
                else:
                    self.motion_dict[key] = "moving"
                
                self.state_x_dict[key] = hysteresis(abs(veh_spd_x + self.rel_spd_lon_dict[key]), self.state_x_dict[key])
                self.state_y_dict[key] = hysteresis(abs(veh_spd_y + self.rel_spd_lat_dict[key]), self.state_y_dict[key])
                
                if self.status_dict[key] == "new":
                    self.motion_confidence_dict[key] = self.motion_confidence_dict[key]
                elif ((self.pos_x_dict[key] > -1.6 and self.pos_x_dict[key] < 10)
                      and (self.pos_y_dict[key] > -8 and self.pos_y_dict[key] < 0)):
                    self.motion_confidence_dict[key] = min(0.9, self.motion_confidence_dict[key] + 0.03)
                else:
                    self.motion_confidence_dict[key] = max(0.03, self.motion_confidence_dict[key] - 0.1)      
                    
            # self.id_list = self.df_pp["id"].values
            # self.df_pp["motion"] = [self.motion_dict[i] for i in self.id_list]
            # self.df_pp["motion_confidence"] = [self.motion_confidence_dict[i] for i in self.id_list]
            ####################################################
            # code improvement
            self.id_list = list(self.d_pp.keys())
            for k in self.id_list:
                update_dict = {"motion": self.motion_dict[k],
                               "motion_confidence":
                                   self.motion_confidence_dict[k]}
                update_dict_in_dict_value_for_key(self.d_pp, update_dict, [k])
            # print ("_get_agent_motion_status_dict orig: ", self.df_pp, len(self.df_pp))
            # print ("_get_agent_motion_status_dict impr: ", self.d_pp, len(self.d_pp))
            ####################################################

    def _get_agent_motion_status(self, veh_spd_x, veh_spd_y): # not in use
        self.df_pp["motion"] = ""
        if len(self.df_pp) > 0:

            self.df_pp["motion"] = (
                self.df_pp.apply(lambda x:
                                  "stationary"
                                  if (hysteresis(abs(veh_spd_x + x["rel_spd_lon"]), x["rel_spd_lon_last_state"])
                                      and 
                                      hysteresis(abs(veh_spd_y + x["rel_spd_lat"]),  x["rel_spd_lat_last_state"]))
                                  else "moving", axis=1))
            self.df_pp["rel_spd_lon_last_state"] = self.df_pp.apply(lambda x:
                                                                    hysteresis(abs(veh_spd_x + x["rel_spd_lon"]), x["rel_spd_lon_last_state"]), axis = 1)
            self.df_pp["rel_spd_lat_last_state"] = self.df_pp.apply(lambda x:
                                                                    hysteresis(abs(veh_spd_y + x["rel_spd_lat"]),  x["rel_spd_lat_last_state"]), axis = 1)
    
            self.df_pp["motion_confidence"] = (
                self.df_pp.apply(lambda x:
                                 x["motion_confidence"]
                                 if x["status"] == "new"
                                 else  min(0.9, x["motion_confidence"] + 0.03)
                                 if ((x["pos_x"] > -1.6 and x["pos_x"] < 10)
                                     and
                                     (x["pos_y"] > -8 and x["pos_y"] < 0))
                                 else max(0.03, x["motion_confidence"] - 0.1),
                                 axis=1))
            
            # self.df_pp["motion_status"] = (
            #     self.df_pp.apply(lambda x:
            #                       "stationary"
            #                       if ((abs(veh_spd_x + x["rel_spd_lon"]) < 0.5)
            #                           and 
            #                           (abs(veh_spd_y + x["rel_spd_lat"]) < 0.5))
            #                       else "moving", axis=1))

            # print(self.df_pp["motion_confidence"])

            # self.df_pp["motion_confidence"] = (
            #     self.df_pp.apply(lambda x:
            #                      x["motion_confidence"]
            #                      if ((x["pos_x"] > -2 and x["pos_x"] < 10)
            #                          and
            #                          (x["pos_y"] > -6 and x["pos_y"] < 0))
            #                      else max(0.03,
            #                               x["motion_confidence"] - 0.03),
            #                      axis=1))

            # print(self.df_pp["motion_confidence"])
        
    def main(self):
        t1=time.time()
        if len(self.h_data) > 1:
            self.d_pp = {}
            self._filter_and_extrapolation()
            # self._update_kf_container()  # legacy
            self._destroy_overlap_ghost_obj()
            self._check_n_update_class()
            self._update_state_xy_dict()
            self._get_agent_motion_status_dict(self.veh_spd_x, self.veh_spd_y)
            # self._update_confid_container()
        else:
            # self.df_pp = self.df
            # self.df_pp["status"] = "new"
            # self.df_pp["class_orig_cam"] = self.df_pp["class"]
            ####################################################
            # code improvement
            self.d_pp = {k: v.copy() for k, v in self.d.items()}
            update_dict_in_dict_value(self.d_pp, {"status": "new"})
            for k in self.d_pp:
                self.d_pp[k].update({"class_orig_cam": self.d_pp[k]["class"]})
            ####################################################
            # if len(self.df_pp) > 0:
            if len(self.d_pp) > 0:
                self._create_kf_filter_in_dict()
                self._update_pos_spd_dict()
                self._create_motion_status_confid_in_dict()
                self._get_agent_motion_status_dict(self.veh_spd_x, self.veh_spd_y)
        t2 = time.time()
        # print ("cam_pp_main_time: ", (t2-t1)*1000)
            #================================================================
            #使用df完成KMfilter和MotionStatus+MotionConfidence
            # self._create_kf_filter_in_df(self.df_pp)  # legacy
            # self._update_kf_container()  # legacy
            # self._update_pos_spd()  # legacy
            # self._create_motion_status_confid_in_df(self.df_pp)# legacy
            # self._get_agent_motion_status(self.veh_spd_x, self.veh_spd_y)# legacy
            # self._update_confid_container()# legacy



def get_img_pathes(path, processed_pic: bool = False):
    imgpathes = []
    if processed_pic:
        path = path + "_data_img"
        for file in sorted(os.listdir(path)):
            imgpath = "/".join([path, file])
            imgpathes.append(imgpath)
    else:
        for folder in sorted(os.listdir(path)):
            files = os.listdir("/".join([path, folder]))
            for file in files:
                if file.find("jpg") >= 0:
                    imgname = file
                    imgpath = "/".join([path, folder, imgname])
                    imgpathes.append(imgpath)
    return imgpathes

def get_tyre_angle(steer_angle: float):
    steer_angle_range = [-15.71, -12.57, 0, 12.57, 15.71]
    tyre_angle_range = [30, 28, 27, 28, 29]
    tyre_angle = steer_angle / (np.interp(steer_angle,
                                               steer_angle_range,
                                               tyre_angle_range))
    return tyre_angle