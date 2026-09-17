# -*- coding: utf-8 -*-
"""
Created on Tue Sep 12 15:27:28 2023

@author: Z0030883
"""
from collections import deque
import math
from scipy.io import loadmat
from typing import SupportsFloat as Numeric

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from ego_state.unit_converter import UnitConverter
from ego_state.config import Config
# from unit_converter import UnitConverter
# from config import Config

# import line_profiler
import sys


def sort_small(df, col, by: str = "Priority"):
    """
    Sort the table based on a given column in ascending order, and get the
    value of the specified column(s) that is(are) in the first row."""
    return df.sort_values(by=by, ascending=True).iloc[0][col]


def sort_large(df, col: str, by: str = "Confid") -> pd.DataFrame:
    return df.sort_values(by=by, ascending=False).iloc[0][col]


def filter_move_average(signal: Numeric, window: deque) -> Numeric:
    """
    Filter the given signal by moving average method.

    Parameters
    ----------
    signal : Numeric
        The signal which is needed to be filtered.
    window : deque
        The moving average window, which is a FIFO with a dedicated length .

    Returns
    -------
    Numeric
        The filtered signal by get the mean value of the new and historic data.

    """
    window.append(signal)
    return np.array(window).mean()


class Plot:

    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(16, 8), dpi=300)

    def plot_articulation_angle(self,
                                x_array: np.ndarray,
                                y_array: np.ndarray):
        self.ax.plot(x_array, y_array, color="red",
                     alpha=0.8, lw=1, label="articulation_angle")

    def plot_articulation_angle_meas(self, x_array: np.ndarray,
                                     y_array: np.ndarray):
        self.ax.plot(x_array, y_array, color="green",
                     alpha=0.4, lw=1, label="articulation_angle_measured")

    def show_plot(self):
        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("value")
        self.ax.set_autoscalex_on(True)
        # self.ax.set_xlim(0, 350)
        self.ax.set_autoscaley_on(True)
        # self.ax.set_ylim(-0.2, 0.5)
        self.ax.legend()
        plt.show()


class VehSpd:
    """This class calculates the vehicle speed based on 4 sources"""

    def __init__(self, es):
        """
        An instance "es" of EgoState class is passed as a parameter to this
        function, aiming to pass the paramters from es into this VehSpd Class.
        """
        
        if speed_source == 0:
            self.speed_df = es.speed_df  # need to be returned
        else:
            self.speed_dict = es.speed_dict
        self.hrw_speed_fl = es.hrw_speed_fl
        self.hrw_speed_fr = es.hrw_speed_fr
        self.tsmo_rpm = es.tsmo_rpm
        self.tsmi_rpm = es.tsmi_rpm
        self.tsmi_rpm_prev = es.tsmi_rpm_prev
        self.tsmi_rpm_uplimit = es.tsmi_rpm_uplimit
        self.tach_speed = es.tach_speed
        self.veh_speed = es.veh_speed  # need to be returned
        self.veh_speed_prev = es.veh_speed_prev  # need to be returned
        self.unit_conv = es.unit_conv
        self.eng2whl_ratio = es.eng2whl_ratio
        self.priority_table = es.priority_table
        self.gear_ratio = es.gear_ratio
        self.gear_ratio_prev = es.gear_ratio_prev
        self.gear_ratio_uplimit = es.gear_ratio_uplimit
        self.current_gear = es.current_gear
        self.current_gear_prev = es.current_gear_prev
        self.shift_process = es.shift_process
        self.veh_speed_downgrade = es.veh_speed_downgrade

       
    def get_veh_speed(self):
        """
        This is the core function of the VehSpd Class. It calls the functions
        below one by one.
        Firstly, it updates the latest speed signals from different sources
        into the dataframe except tsmi speed. As an aternative, tsmi rpm is
        temporarily used to replace.
        Secondly, check the signal validity for each speed source.
        Thirdly, the tsmi speed is calculated and replace the tsmi rpm in the
        dataframe.
        An "if else" is used in case all signals are not valid, the vehicle
        speed should keep as last value. Otherwise, the following calculation
        should be continued. All the valid speed values are used to calculate
        a criteria speed. The criteria speed is used to choose different
        priority ranking for all sources. Select the valid speed with highest
        priority as the final vehicle speed at last.
        """
        self._gear_ratio_protect()
        self._speed_update()
        self._determine_speed_validity()
        self._tsmi_speed_protect()

        # 防止所有的信号检查完Validity后都不满足，从而后续计算找不到值报错
        if self.speed_df["Validity"].sum() == 0:  # none signal is valid
            self.veh_speed = self.veh_speed_prev
            self.veh_speed_downgrade = 1
        else:
            self._get_criteria_speed()
            self._define_speed_priority()
            self._determine_veh_speed()
            self.veh_speed_prev = self.veh_speed
            
    def get_veh_speed_dict(self):
        """
        This is the core function of the VehSpd Class. It calls the functions
        below one by one.
        Firstly, it updates the latest speed signals from different sources
        into the dataframe except tsmi speed. As an aternative, tsmi rpm is
        temporarily used to replace.
        Secondly, check the signal validity for each speed source.
        Thirdly, the tsmi speed is calculated and replace the tsmi rpm in the
        dataframe.
        An "if else" is used in case all signals are not valid, the vehicle
        speed should keep as last value. Otherwise, the following calculation
        should be continued. All the valid speed values are used to calculate
        a criteria speed. The criteria speed is used to choose different
        priority ranking for all sources. Select the valid speed with highest
        priority as the final vehicle speed at last.
        """
        self._gear_ratio_protect()
        self._speed_update_dict()
        self._determine_speed_validity_dict()
        self._tsmi_speed_protect_dict()

        # 防止所有的信号检查完Validity后都不满足，从而后续计算找不到值报错
        if not any([value["Validity"] for value in self.speed_dict.values()]):  # none signal is valid
            self.veh_speed = self.veh_speed_prev
            self.veh_speed_downgrade = 1
        else:
            self._get_criteria_speed_dict()
            self._define_speed_priority_dict()
            self._determine_veh_speed_dict()
            self.veh_speed_prev = self.veh_speed

    def _speed_update(self):
        self._get_speed_from_sources()
        self.speed_df["Speed"] = [
            self.hrw_speed, self.tsmo_speed, self.tsmi_rpm, self.tach_speed
        ]
    
    def _speed_update_dict(self):
        self._get_speed_from_sources()
        for key, value in self.speed_dict.items():
            if key == "HRW":
                self.speed_dict[key]["Speed"] = self.hrw_speed
            elif key == "TSMO":
                self.speed_dict[key]["Speed"] = self.tsmo_speed
            elif key == "TSMI":
                self.speed_dict[key]["Speed"] = self.tsmi_rpm
            elif key == "TACH":
                self.speed_dict[key]["Speed"] = self.tach_speed
            else:
                self.speed_dict[key]["Speed"] = 0.0

    def _get_speed_from_sources(self):
        self.hrw_speed = (self.hrw_speed_fl + self.hrw_speed_fr) / 2
        self.tsmo_speed = self.tsmo_rpm * self.unit_conv.tsmo2kph

    def _gear_ratio_protect(self):
        """This method must be called before using 'self.gear_ratio' OR embed
        this function in message receive callback. This method is used to
        keep the gear ratio during the gear change period.
        """
        # if self.gear_ratio > self.gear_ratio_uplimit:
        if self.current_gear == 0 or self.shift_process == 1:
            self.gear_ratio = self.gear_ratio_prev
        else:
            self.gear_ratio_prev = self.gear_ratio

    def _determine_speed_validity(self):
        """This method checks if the speed value is in the valid range, then
        the Validirty column is set to True, otherwise the Validity column
        keeps as False. """
        self.speed_df["Validity"] = False
        self.speed_df.loc[((self.speed_df["Speed"] >= self.speed_df["Min"]) &
                           (self.speed_df["Speed"] <= self.speed_df["Max"])),
                          "Validity"] = True
    
    def _determine_speed_validity_dict(self):   # 0730
        """This method checks if the speed value is in the valid range, then
        the Validirty column is set to True, otherwise the Validity column
        keeps as False. """
        for key, value in self.speed_dict.items():
            self.speed_dict[key]["Validity"] = False
            if ((self.speed_dict[key]["Speed"] >= self.speed_dict[key]["Min"])
                &
                (self.speed_dict[key]["Speed"] <= self.speed_dict[key]["Max"])):
                self.speed_dict[key]["Validity"] = True
                
    def _tsmi_speed_protect(self):
        """This method calculates the tsmi speed if eng2whl_ratio is calculated
        successfully, otherwise, tsmi_speed is set to 0, and the Validity is
        set to False"""
        # if self.tsmi_rpm > self.tsmi_rpm_uplimit:
        if self.shift_process:
            self.tsmi_rpm = self.tsmi_rpm_prev
        else:
            self.tsmi_rpm_prev = self.tsmi_rpm

        if self.eng2whl_ratio > 0 and self.current_gear != 0 and self.shift_process == 0:
            # self.tsmi_speed = (self.tsmi_rpm /
            #                    self.eng2whl_ratio *
            #                    self.unit_conv.rpm2kph_wo_radius)
            self.tsmi_speed = (
                self.tsmi_rpm /
                (self.gear_ratio * self.unit_conv.diffrntl_ratio /
                 self.unit_conv.tyre_radius) *
                self.unit_conv.rpm2kph_wo_radius)

            self.speed_df.loc["TSMI", "Validity"] = True
        else:
            self.tsmi_speed = 0.
            self.speed_df.loc["TSMI", "Validity"] = False
        self.speed_df.loc["TSMI", "Speed"] = self.tsmi_speed
        
    def _tsmi_speed_protect_dict(self):
        """This method calculates the tsmi speed if eng2whl_ratio is calculated
        successfully, otherwise, tsmi_speed is set to 0, and the Validity is
        set to False"""
        # if self.tsmi_rpm > self.tsmi_rpm_uplimit:
        if self.shift_process:
            self.tsmi_rpm = self.tsmi_rpm_prev
        else:
            self.tsmi_rpm_prev = self.tsmi_rpm

        if self.eng2whl_ratio > 0 and self.current_gear != 0 and self.shift_process == 0:
            # self.tsmi_speed = (self.tsmi_rpm /
            #                    self.eng2whl_ratio *
            #                    self.unit_conv.rpm2kph_wo_radius)
            self.tsmi_speed = (
                self.tsmi_rpm /
                (self.gear_ratio * self.unit_conv.diffrntl_ratio /
                 self.unit_conv.tyre_radius) *
                self.unit_conv.rpm2kph_wo_radius)

            self.speed_dict["TSMI"]["Validity"] = True
        else:
            self.tsmi_speed = 0.
            self.speed_dict["TSMI"]["Validity"] = False
        self.speed_dict["TSMI"]["Speed"] = self.tsmi_speed

    def _get_criteria_speed(self):
        """This method calculates the mean value of all valid speed as the
        criteria speed signal"""
        self.crit_speed = self.speed_df[self.speed_df["Validity"] !=
                                        0]["Speed"].mean()
        
    def _get_criteria_speed_dict(self):
        valid_speed_list = []
        for key, value in self.speed_dict.items():
            if value["Validity"] == True:
                valid_speed_list.append(value["Speed"])
        self.crit_speed = sum(valid_speed_list) / len(valid_speed_list)

    def _define_speed_priority(self):
        """This method creates a new dataframe and stores the relationship
        between the priority list and criteria speed. There are 3 speed ranges,
        <1, 1-3, >3. For each speed range, the priority for 4 speed sources
        differs, and the different priorities are stored into 3 lists.
        According to the criteria speed, choose the corresponding priority for
        4 speed sources. Finally, the choosen priority list is written to the
        "Priority" column of speed dataframe.
        """
        self.priority_speed_df = pd.DataFrame(self.priority_table)
        select = ((self.crit_speed >= self.priority_speed_df["min_speed"]) &
                  (self.crit_speed < self.priority_speed_df["max_speed"]))
        # 检查select条件筛选后，防止没有一个条件满足从而返回空的值，从而使用iloc报
        # 错。但是11.2号检查感觉这个没必要，因为必然落在一个条件里,保留也行，有这样
        # 的意识也好
        priority_list = (self.priority_speed_df[select]["priority_list"])
        if len(priority_list) != 0:
            self.speed_df["Priority"] = priority_list.iloc[0]
            
    def _define_speed_priority_dict(self):
        """This method creates a new dataframe and stores the relationship
        between the priority list and criteria speed. There are 3 speed ranges,
        <1, 1-3, >3. For each speed range, the priority for 4 speed sources
        differs, and the different priorities are stored into 3 lists.
        According to the criteria speed, choose the corresponding priority for
        4 speed sources. Finally, the choosen priority list is written to the
        "Priority" column of speed dataframe.
        """

        priority_list = []
        for index in range(0, len(self.priority_table["min_speed"])):
            if ((self.crit_speed >= self.priority_table["min_speed"][index]) &
                (self.crit_speed < self.priority_table["max_speed"][index])):
                priority_list = self.priority_table["priority_list"][index]
           
        # 检查select条件筛选后，防止没有一个条件满足从而返回空的值，从而使用iloc报
        # 错。但是11.2号检查感觉这个没必要，因为必然落在一个条件里,保留也行，有这样
        # 的意识也好
        index_list = ["HRW", "TSMO", "TSMI", "TACH"]
        if len(priority_list) != 0:
            for index, value in enumerate(index_list):
                self.speed_dict[value]["Priority"] = priority_list[index]

    def _determine_veh_speed(self):
        """This method pick the valid speed values as a new filtered dataframe,
        sort the new dataframe by the priority, and return the speed value with
        highest priority(lowest value)"""
        self.speed_df_filt = self.speed_df[self.speed_df["Validity"] != 0]
        self.veh_speed = self.speed_df_filt.sort_values(
            by="Priority", ascending=True)["Speed"][0]

        # if self.current_gear == 0:
        #     self.current_gear = self.current_gear_prev
        # else:
        #     self.current_gear_prev = self.current_gear
        if self.current_gear < 0:
            self.veh_speed = -1 * self.veh_speed

    def _determine_veh_speed_dict(self):
        """This method pick the valid speed values as a new filtered dataframe,
        sort the new dataframe by the priority, and return the speed value with
        highest priority(lowest value)"""

        highest_priority = max(self.priority_table["priority_list"][0]) + 1
        for key, value in self.speed_dict.items():
            if value["Validity"] == True:
                if value["Priority"] < highest_priority:
                    highest_priority = value["Priority"]
                    highest_priority_key = key
        self.veh_speed = self.speed_dict[highest_priority_key]["Speed"]

        if self.current_gear < 0:
            self.veh_speed = -1 * self.veh_speed

class VehWght:
    """This class calculates the vehicle weight based on 2 sources"""

    def __init__(self, es):
        """
        An instance "es" of EgoState class is passed as a parameter to this
        function, aiming to pass the paramters from es into this VehWght Class.
        """
        if weight_source == 0:
            self.weight_df = es.weight_df  # need to be returned
        else:
            self.weight_dict = es.weight_dict
        self.veh_weight = es.veh_weight  # need to be returned
        self.veh_weight_prev = es.veh_weight_prev  # need to be returned
        self.weight_amt = es.weight_amt
        self.weight_ebs = es.weight_ebs
        self.weight_priority_table_dict = es.weight_priority_table_dict
        self.veh_weight_downgrade = es.veh_weight_downgrade

    def get_veh_weight(self):
        """
        This is the core function of the VehWght Class. It calls the functions
        below one by one.
        Firstly, it updates the latest weight signals from different sources
        into the dataframe.
        Secondly, check the signal validity for each speed source.
        An "if else" is used in case all signals are not valid, the vehicle
        speed should keep as last value. Otherwise, the following calculation
        should be continued."""
        self._weight_update()
        self._determine_weight_validity()
        # 防止所有的信号检查完Validity后都不满足，从而后续计算找不到值报错
        if self.weight_df["Validity"].sum() == 0:
            self.veh_weight = self.veh_weight_prev
            self.veh_weight_downgrade = 1
        else:
            self._determine_veh_weight()
            self.veh_weight_prev = self.veh_weight
            
    def get_veh_weight_dict(self):
        """
        This is the core function of the VehWght Class. It calls the functions
        below one by one.
        Firstly, it updates the latest weight signals from different sources
        into the dataframe.
        Secondly, check the signal validity for each speed source.
        An "if else" is used in case all signals are not valid, the vehicle
        speed should keep as last value. Otherwise, the following calculation
        should be continued."""
        self._weight_update_dict()
        self._determine_weight_validity_dict()
        # 防止所有的信号检查完Validity后都不满足，从而后续计算找不到值报错
        if not any([value["Validity"] for value in self.weight_dict.values()]):
            self.veh_weight = self.veh_weight_prev
            self.veh_weight_downgrade = 1
        else:
            self._determine_veh_weight_dict()
            self.veh_weight_prev = self.veh_weight

    def _weight_update(self):
        self.weight_df["Weight"] = [self.weight_amt, self.weight_ebs]

    def _weight_update_dict(self):
        for key, value in self.weight_dict.items():
            if key == "CVW_AMT":
                self.weight_dict[key]["Weight"] = self.weight_amt
            elif key == "CVW_EBS":
                self.weight_dict[key]["Weight"] = self.weight_ebs
            else:
                self.weight_dict[key]["Weight"] = 0.0

    def _determine_weight_validity(self):
        """This method checks if the weight value is in the valid range, then
        the Validirty column is set to True, otherwise the Validity column
        keeps as False. """
        self.weight_df["Validity"] = False
        self.weight_df.loc[(self.weight_df["Weight"] > self.weight_df["Min"]) &
                           (self.weight_df["Weight"] < self.weight_df["Max"]),
                           "Validity"] = True
        if self.weight_df.loc["CVW_AMT", "Weight"] == 20000.:
            self.weight_df.loc["CVW_AMT", "Validity"] = False
            
    def _determine_weight_validity_dict(self):
        """This method checks if the weight value is in the valid range, then
        the Validirty column is set to True, otherwise the Validity column
        keeps as False. """
        for key, value in self.weight_dict.items():
            value["Validity"] = False
            if ((value["Weight"] > value["Min"]) and (value["Weight"] < value["Max"])):
                value["Validity"] = True
        if self.weight_dict["CVW_AMT"]["Weight"] == 20000.:
            self.weight_dict["CVW_AMT"]["Validity"] = False

    def _determine_veh_weight(self):
        """This method pick the valid weight values as a new filtered dataframe
        , sorts the new dataframe by the priority, and return the weight value
        with highest priority(lowest value)"""
        self.weight_df_filter = self.weight_df[self.weight_df["Validity"] != 0]
        self.veh_weight = self.weight_df_filter.sort_values(
            by="Priority", ascending=True)["Weight"][0]

    def _determine_veh_weight_dict(self):
        """This method pick the valid weight values as a new filtered dataframe
        , sorts the new dataframe by the priority, and return the weight value
        with highest priority(lowest value)"""

        highest_priority = max(self.weight_priority_table_dict.values()) + 1
        for key, value in self.weight_dict.items():
            if value["Validity"] == True:
                if value["Priority"] < highest_priority:
                    highest_priority = value["Priority"]
                    highest_priority_key = key
        self.veh_weight = self.weight_dict[highest_priority_key]["Weight"]


# =============================================================================
# Calculate Articulation Angle by Kinematics
class ArtAng:
    """This class has two main functions, the 1st one is calculating artic
    angle by vehicle kinematic equation and the 2nd one is fusing artic angle
    from 3 sources (vehicle kinematic estimation, the radar detection,
    the cv detection)"""

    def __init__(self, es):
        """
        An instance "es" of EgoState class is passed as a parameter to this
        method, aiming to pass the paramters from es into this ArtAng Class.
        """
        # self.time = es.time
        # self.new_mat = es.new_mat
        self.artic_angle_vd = es.artic_angle_vd  # need2return
        self.artic_angle_confid_vd_decel = es.artic_angle_confid_vd_decel
        self.artic_angle_confid_vd = es.artic_angle_confid_vd  # need2return
        self.artic_angle_confid_vd_lwlimit = es.artic_angle_confid_vd_lwlimit
        self.wheel_angle = es.wheel_angle
        self.wheel_angle_strght_uplimit = es.wheel_angle_strght_uplimit
        self.forward_drive_distance = es.forward_drive_distance  # need2return
        self.time_temp = es.time_temp
        self.forward_drive_distance_lwlimit = es.forward_drive_distance_lwlimit
        self.veh_speed = es.veh_speed
        self.tractor_yawrate = es.tractor_yawrate
        self.hitch2rear_axle = es.hitch2rear_axle
        self.trailer_wheelbase = es.trailer_wheelbase
        self.unit_conv = es.unit_conv
        self.config = es.config
        # The below signals are for fusion
        self.artic_angle_prev = es.artic_angle_prev
        self.artic_angle = es.artic_angle
        if artic_angle_source == 0:
            self.artic_angle_df = es.artic_angle_df
        else:
            self.artic_angle_dict = es.artic_angle_dict
        self.artic_angle_sr = es.artic_angle_sr
        self.artic_angle_confid_sr = es.artic_angle_confid_sr
        self.artic_angle_cv = es.artic_angle_cv
        self.artic_angle_confid_cv = es.artic_angle_confid_cv
        self.artic_angle_min = es.artic_angle_min
        self.artic_angle_max = es.artic_angle_max

        self.artic_angle_downgrade = es.artic_angle_downgrade
        self.current_gear = es.current_gear
        self.angle_priority_table = es.angle_priority_table

    def calc_artic_angle(self):
        """
        This method calculates the articulation angle by vehicle kinematic(vd)
        estimation. There are two steps, the 1st step is to calculate the yaw
        rate of trailer, and the 2nd step is to update the articulation angle.
        The "_reset_artic_angle" method is called to reset the articulation
        angle to 0 when the steering wheel angle is small which means straight
        driving for a long enough distance.
        Moreover, the articulation angle confidence from vd estimation
        is also decresed until to the min value, because the error is getting
        bigger with more steps.

        """
        dt = self.config.ts
        # 计算trailer_yawrate(for ROS2 Use)
        self.trailer_yawrate = (self.veh_speed
                                * self.unit_conv.kph2mps
                                * math.sin(self.artic_angle_vd)
                                + self.tractor_yawrate
                                * self.hitch2rear_axle
                                * math.cos(self.artic_angle_vd)
                                ) / self.trailer_wheelbase
        # 更新θ
        self.artic_angle_vd += (self.tractor_yawrate
                                - self.trailer_yawrate
                                ) * dt

        self.artic_angle_confid_vd += self.artic_angle_confid_vd_decel
        self.artic_angle_confid_vd = max(self.artic_angle_confid_vd_lwlimit,
                                         self.artic_angle_confid_vd)
        
        if self.current_gear < 0:
            self.artic_angle_vd = 0.0
            self.artic_angle_confid_vd = 0.0

        self._reset_artic_angle(dt)

    def _reset_artic_angle(self, time_step: float):
        """This method integrate the driving distance under the condition that
        vehicle is driving straight(steering wheel angle smaller than a small
        range). Once the steering wheel angle is getting big, the calculation
        should restart. Finally, if the straight driving for a specified
        distance is fulfilled, the articulation angle is set to 0, and the
        estimated angle confidence is set to 1.0(100%)."""
        if abs(self.wheel_angle) < self.wheel_angle_strght_uplimit:
            self.forward_drive_distance += (self.veh_speed
                                            * self.unit_conv.kph2mps
                                            * time_step)
            # self.time_temp += time_step
            # print(self.forward_drive_distance, end="/")
            # print(self.time_temp)
        else:
            self.forward_drive_distance = 0.

        if self.forward_drive_distance > self.forward_drive_distance_lwlimit:
            self.artic_angle_vd = 0.
            self.artic_angle_confid_vd = 0.9

    # def calc_artic_angle(self):
    #     angle_result = np.array([])
    #     t_pre = 0.

    #     # 如果上ROS，就把For里的东西搬出来即可
    #     for indx, t in enumerate(self.time):
    #         dt = t - t_pre
    #         # 计算trailer_yawrate(for offline test),注意这里的速度是mps!!!!!!!
    #         self.trailer_yawrate = (self.new_mat["Vehicle_Speed"][indx] *
    #                                 math.sin(self.artic_angle_vd) +
    #                                 self.new_mat["Yaw_Rate"][indx] *
    #                                 self.new_mat["L_1"][indx] *
    #                                 math.cos(self.artic_angle_vd)
    #                                 ) / self.new_mat["L_2"][indx]
    #         # 更新θ
    #         self.artic_angle_vd += (self.new_mat["Yaw_Rate"][indx] -
    #                                 self.trailer_yawrate) * dt

    #         self.artic_angle_confid_vd += self.artic_angle_confid_vd_decel
    #         self.artic_angle_confid_vd = max(
    #                                      self.artic_angle_confid_vd_lwlimit,
    #                                      self.artic_angle_confid_vd)
    #         # print(self.artic_angle_confid_vd)
    #         t_pre = t
    #         self._reset_artic_angle(indx, dt)
    #         angle_result = np.append(angle_result, self.artic_angle_vd)

    #     print(t_pre)
    #     return angle_result

    # def _reset_artic_angle(self, index: int, time_step: float):
    #     if abs(self.wheel_angle[index]) < self.wheel_angle_strght_uplimit:
    #         self.forward_drive_distance += (self.new_mat["Vehicle_Speed"][index]
    #                                    ) * time_step
    #         self.time_temp += time_step
    #         print(self.forward_drive_distance, end="/")
    #         print(self.time_temp)

    #     else:
    #         self.forward_drive_distance = 0

    #     if self.forward_drive_distance > self.forward_drive_distance_lwlimit:
    #         self.artic_angle_vd = 0
    #         self.artic_angle_confid_vd = 1.0

# =============================================================================
# Articulation Angle Fusion

    def fuse_artic_angle(self):
        """This is the core fusion function of the ArtAng Class.
        Firstly, it updates the latest articulation angle signals and their
        confidence values from different sources into the dataframe.
        Secondly, check the signal validity for each articulation angle source.
        An "if else" is used in case all signals are not valid, the vehicle
        speed should keep as last. Otherwise, the "_determine_artic_angle"
        method in the "else" should be continued to get the final articulation
        angle."""
        self._artic_angle_update()
        self._determine_angle_validity()
        # 防止所有的信号检查完Validity后都不满足，从而后续计算找不到值报错
        if self.artic_angle_df[self.artic_angle_df["Validity"]].empty:
            self.artic_angle = self.artic_angle_prev
            self.artic_angle_downgrade = 1
        else:
            self._determine_artic_angle()
            self.artic_angle_prev = self.artic_angle

    def fuse_artic_angle_dict(self):
        """This is the core fusion function of the ArtAng Class.
        Firstly, it updates the latest articulation angle signals and their
        confidence values from different sources into the dataframe.
        Secondly, check the signal validity for each articulation angle source.
        An "if else" is used in case all signals are not valid, the vehicle
        speed should keep as last. Otherwise, the "_determine_artic_angle"
        method in the "else" should be continued to get the final articulation
        angle."""
        self._artic_angle_update_dict()
        self._determine_angle_validity_dict()
        # 防止所有的信号检查完Validity后都不满足，从而后续计算找不到值报错
        if not any([value["Validity"] for value in self.artic_angle_dict.values()]):
            self.artic_angle = self.artic_angle_prev
            self.artic_angle_downgrade = 1
        else:
            self._determine_artic_angle_dict()
            self.artic_angle_prev = self.artic_angle

    def _artic_angle_update(self):
        self.artic_angle_df["Angle"] = [
                                        self.artic_angle_sr,
                                        self.artic_angle_vd,
                                        self.artic_angle_cv
                                        ]

        self.artic_angle_df["Confid"] = [
                                         self.artic_angle_confid_sr,
                                         self.artic_angle_confid_vd,
                                         self.artic_angle_confid_cv
                                         ]
        
    def _artic_angle_update_dict(self):
        for key, value in self.artic_angle_dict.items():
            if key == "SR":
                self.artic_angle_dict[key]["Angle"] = self.artic_angle_sr
                self.artic_angle_dict[key]["Confid"] = self.artic_angle_confid_sr
            elif key == "VD":
                self.artic_angle_dict[key]["Angle"] = self.artic_angle_vd
                self.artic_angle_dict[key]["Confid"] = self.artic_angle_confid_vd
            elif key == "CV":
                self.artic_angle_dict[key]["Angle"] = self.artic_angle_cv
                self.artic_angle_dict[key]["Confid"] = self.artic_angle_confid_cv
            else:
                self.artic_angle_dict[key]["Angle"] = 0.0
                self.artic_angle_dict[key]["Confid"] = 0.0
        
    def _determine_angle_validity(self):
        """This method checks if the angle value is in the valid range, then
        the Validirty column is set to True, otherwise the Validity column
        keeps as False. """
        self.artic_angle_df["Validity"] = False
        self.artic_angle_df.loc[
            (self.artic_angle_df["Angle"] > self.artic_angle_min) &
            (self.artic_angle_df["Angle"] < self.artic_angle_max),
            "Validity"] = True

        if self.current_gear < 0:
            self.artic_angle_df.loc["VD", "Validity"] = False

            
    def _determine_angle_validity_dict(self):
        """This method checks if the angle value is in the valid range, then
        the Validirty column is set to True, otherwise the Validity column
        keeps as False. """
        for key, value in self.artic_angle_dict.items():
            value["Validity"] = False
            if ((value["Angle"] > self.artic_angle_min)
                and 
                (value["Angle"] < self.artic_angle_max)):
                value["Validity"] = True

        if self.current_gear < 0:
            self.artic_angle_dict["VD"]["Validity"] = False



    def _determine_artic_angle(self):
        """This method is the core method to fuse the articulation angles from
        different source.
        Firstly there are two conditions defined to select the satisfied
        rows from DataFrame.
        'high_confid_threshold' selects the valid source rows with high confidences.
        'low_confid_threshold' selects the valid source rows with low confidences.
        "If - elif - else" are used to calculate angles under 3 conditions.
        If at least 1 row fulfills the 'high_confid_threshold' criteria, find
        the row with highest priority.
        If the first condition is not fulfilled, then check the second one
        that if at least 2 rows fulfull the 'low_confid_threshold' condition,
        and get the mean value of the remaining rows.
        If all the valid rows have a very low confidence which cannot fulfill
        the upper two conditions or only one row fulfills the second condition,
        find the row which has the closest angle value with last step.
        """
        high_confid_threshold = ((self.artic_angle_df["Validity"]) &
                                 (self.artic_angle_df["Confid"] >= 0.5))
        low_confid_threshold = ((self.artic_angle_df["Validity"]) &
                                (self.artic_angle_df["Confid"] >= 0.2))
        valid_select = (self.artic_angle_df["Validity"])
        if not self.artic_angle_df[high_confid_threshold].empty:
            artic_angle_df_filter = self.artic_angle_df[high_confid_threshold]
            self.artic_angle, self.artic_angle_confid = sort_small(
                artic_angle_df_filter, ["Angle", "Confid"], "Priority")
            # print("Condition 1")

        elif self.artic_angle_df[low_confid_threshold].shape[0] > 1:
            artic_angle_df_filter = self.artic_angle_df[low_confid_threshold]
            self.artic_angle = artic_angle_df_filter["Angle"].mean()
            self.artic_angle_confid = artic_angle_df_filter["Confid"].mean()
            # print("Condition 2")
        else:
            artic_angle_df_filter = self.artic_angle_df[valid_select]
            idx = np.argmin(
                np.abs(artic_angle_df_filter["Angle"] - self.artic_angle_prev))
            self.artic_angle = artic_angle_df_filter.iloc[idx]["Angle"]
            self.artic_angle_confid = artic_angle_df_filter.iloc[idx]["Confid"]
            # print("Condition 3")

    def _determine_artic_angle_dict(self):
        """This method is the core method to fuse the articulation angles from
        different source.
        Firstly there are two conditions defined to select the satisfied
        rows from DataFrame.
        'high_confid_threshold' selects the valid source rows with high confidences.
        'low_confid_threshold' selects the valid source rows with low confidences.
        "If - elif - else" are used to calculate angles under 3 conditions.
        If at least 1 row fulfills the 'high_confid_threshold' criteria, find
        the row with highest priority.
        If the first condition is not fulfilled, then check the second one
        that if at least 2 rows fulfull the 'low_confid_threshold' condition,
        and get the mean value of the remaining rows.
        If all the valid rows have a very low confidence which cannot fulfill
        the upper two conditions or only one row fulfills the second condition,
        find the row which has the closest angle value with last step.
        """

        if any([value["Validity"] and (value["Confid"] >= 0.5)
                for value in self.artic_angle_dict.values()]):
            highest_priority = max(self.angle_priority_table.values()) + 1
            for key, value in self.artic_angle_dict.items():
                if value["Validity"] and (value["Confid"] >= 0.5):
                    if value["Priority"] < highest_priority:
                        highest_priority = value["Priority"]
                        highest_priority_key = key
            self.artic_angle = self.artic_angle_dict[highest_priority_key]["Angle"]
            self.artic_angle_confid = self.artic_angle_dict[highest_priority_key]["Confid"]
        elif sum([value["Validity"] and (value["Confid"] >= 0.2)
                  for value in self.artic_angle_dict.values()]) > 1:
            artic_angle_list = []
            artic_angle_confid_list = []
            for key, value in self.artic_angle_dict.items():
                if value["Validity"] and (value["Confid"] >= 0.2):
                    artic_angle_list.append(value["Angle"])
                    artic_angle_confid_list.append(value["Confid"])
            self.artic_angle = sum(artic_angle_list) / len(artic_angle_list)
            self.artic_angle_confid = sum(artic_angle_confid_list) / len(artic_angle_confid_list)
        else:
            min_diff = float('inf')
            for key, value in self.artic_angle_dict.items():
                if value["Validity"]:
                    diff = abs(value["Angle"] - self.artic_angle_prev)
                    if diff < min_diff:
                        min_diff = diff
                        best_key = key
            self.artic_angle = self.artic_angle_dict[best_key]["Angle"]
            self.artic_angle_confid = self.artic_angle_dict[best_key]["Confid"]


# =============================================================================
# Calculate Wheel Base by Kinematics
class WheelBase:
    """This class completes the function that estimates the trailer wheelbase
    (trailer_wheelbase in the equation) by using the equation as the one used
    in estimating articulation angle.
    """

    def __init__(self, es):
        # self.new_mat = es.new_mat
        # self.time = es.time
        self.config = es.config
        self.veh_speed = es.veh_speed
        self.unit_conv = es.unit_conv
        self.tractor_yawrate = es.tractor_yawrate
        self.hitch2rear_axle = es.hitch2rear_axle
        self.artic_angle_ini_wb = es.artic_angle_ini_wb
        self.artic_angle_calc_wb = es.artic_angle_calc_wb  # need2return
        self.trailer_yawrate_wb = es.trailer_yawrate_wb  # need2return
        self.trailer_wheelbase_wb = es.trailer_wheelbase_wb  # need2return
        self.trailer_wheelbase_wb_save = es.trailer_wheelbase_wb_save  # need2return
        self.trailer_wheelbase_wb_prev = es.trailer_wheelbase_wb_prev  # need2return
        self.wb_stable_time = es.wb_stable_time  # need to return
        self.trailer_wheelbase_wb_min = es.trailer_wheelbase_wb_min
        self.trailer_wheelbase_wb_max = es.trailer_wheelbase_wb_max
        self.artic_angle_sr = es.artic_angle_sr
        self.artic_angle_confid_sr = es.artic_angle_confid_sr

    def cal_wheelbase_iterated(self):
        """
        This method calculates the trailer wheel base.
        Step1: The trailer_wheelbase is given an assumed value at first, and
        using the assumed trailer_wheelbase to calculate the articulation angle
        Step2: Calculates the error between calculated articulation angle from
        Step1 and the articulation angle from other source with high confidence
        Use the error to tune the trailer_wheelbase assumption until the error
        is small enough.
        Step3: Saves the stable wheelbase which means the wheelbase converge.
        """

        dt = self.config.ts
        # 计算trailer_yawrate
        self.trailer_yawrate_wb = (self.veh_speed
                                   * self.unit_conv.kph2mps
                                   * math.sin(self.artic_angle_calc_wb)
                                   + self.tractor_yawrate
                                   * self.hitch2rear_axle
                                   * math.cos(self.artic_angle_calc_wb)
                                   ) / self.trailer_wheelbase_wb
        # 计算θ
        self.artic_angle_calc_wb += (self.tractor_yawrate
                                     - self.trailer_yawrate_wb
                                     ) * dt
        # 计算误差
        angle_error = self.artic_angle_sr - self.artic_angle_calc_wb

        # 根据误差调整trailer_wheelbase
        if angle_error > 0.03:
            self.trailer_wheelbase_wb += 0.01
        elif angle_error < -0.05:
            self.trailer_wheelbase_wb -= 0.015
        else:
            self.trailer_wheelbase_wb = self.trailer_wheelbase_wb + 0

        self.trailer_wheelbase_wb = max(self.trailer_wheelbase_wb_min,
                                        min(self.trailer_wheelbase_wb,
                                            self.trailer_wheelbase_wb_max))

        self._save_stable_wheelbase(dt)

    def _save_stable_wheelbase(self, dt: float):
        """This method sums the time if the estimated trailer_wheelbase_wb does
        not change and the value is within a valid range. If the summed time is
        longer than 300s, the estimated trailer_wheelbase_wb value is saved."""
        if (self.trailer_wheelbase_wb == self.trailer_wheelbase_wb_prev):
            self.wb_stable_time += dt
        else:
            self.wb_stable_time = 0
        self.trailer_wheelbase_wb_prev = self.trailer_wheelbase_wb
        # refresh the saved value once the consistent valid wheelbase lasts
        # longer than 300s
        if self.wb_stable_time >= 300:
            print(self.wb_stable_time)
            self.trailer_wheelbase_wb_save = self.trailer_wheelbase_wb
            self.wb_stable_time = 0

    # def cal_wheelbase_iterated(self):

    #     # 初始化
    #     t_pre = 0.
    #     trailer_wheelbase_array = np.array([])

    #     # 如果上ROS，就把For里的东西搬出来即可
    #     for indx, t in enumerate(self.time):
    #         dt = t - t_pre

    #         # 计算trailer_yawrate
    #         self.trailer_yawrate_wb = (self.new_mat["Vehicle_Speed"][indx] *
    #                                 math.sin(self.artic_angle_calc_wb) +
    #                                 self.new_mat["Yaw_Rate"][indx] *
    #                                 self.new_mat["L_1"][indx] *
    #                                 math.cos(self.artic_angle_calc_wb)
    #                                 )/self.trailer_wheelbase_wb
    #         # 计算θ
    #         self.artic_angle_calc_wb += (self.new_mat["Yaw_Rate"][indx] -
    #                                       self.trailer_yawrate_wb) * dt
    #         # 计算误差
    #         angle_error = np.abs(self.new_mat["Yaw_Rate_Estimated"][indx] -
    #                               self.artic_angle_calc_wb)

    #         # 根据误差调整trailer_wheelbase
    #         if angle_error > 0.02:
    #             self.trailer_wheelbase_wb += 0.01
    #         elif angle_error < -0.02:
    #             self.trailer_wheelbase_wb -= 0.01
    #         else:
    #             self.trailer_wheelbase_wb = self.trailer_wheelbase_wb + 0
    #         trailer_wheelbase_array = np.append(trailer_wheelbase_array,
    #                                             self.trailer_wheelbase_wb)
    #         t_pre = t
    #         # print(self.artic_angle_calc_wb, end="/")
    #         # print(self.new_mat["Yaw_Rate_Estimated"][indx], end="/")
    #         # print(angle_error)
    #         self._save_stable_wheelbase(dt)

    #     return trailer_wheelbase_array

    # def _save_stable_wheelbase(self, dt: float):
    #     if ((self.trailer_wheelbase_wb == self.trailer_wheelbase_wb_prev)
    #          and (self.trailer_wheelbase_wb_min <=
    #               self.trailer_wheelbase_wb <=
    #               self.trailer_wheelbase_wb_max)):
    #         self.wb_stable_time += dt
    #     else:
    #         self.wb_stable_time = 0
    #     self.trailer_wheelbase_wb_prev = self.trailer_wheelbase_wb
    #     # refresh the saved value once the consistent valid wheelbase lasts
    #     # longer than 300s
    #     if self.wb_stable_time >= 300:
    #         print(self.wb_stable_time)
    #         self.trailer_wheelbase_wb_save = self.trailer_wheelbase_wb
    #         self.wb_stable_time = 0


class TrlrLen:
    """
    This class gets the arbitrated trailer length from two sources (side radar
    and cv) based on 2 trailer length range DataFrames, one for each source.
    The 2 trailer length range DataFrames are generated outside this class, but
    in the EgoState Class, and the update of these two DataFrames are completed
    by two ROS message reception callback functions.
    """

    def __init__(self, es):
        self.len_range_df_sr = es.len_range_df_sr
        self.len_range_df_cv = es.len_range_df_cv
        self.trailer_len_cycle_sr = es.trailer_len_cycle_sr
        self.trailer_len_cycle_cv = es.trailer_len_cycle_cv
        self.confid_sum_threshold = es.confid_sum_threshold
        self.trailer_len_default = es.trailer_len_default
        self.trailer_len_confid_default = es.trailer_len_confid_default
        self.trailer_len = self.trailer_len_default
        self.trailer_len_confid = self.trailer_len_confid_default

    def determine_trailer_length(self):
        """This method has 4 steps:
        Step1: From trailer length range DataFrames, sorts the DataFrame by the
        'Confid' column with descending order, and the first row of DataFrame
        has the highest confidence sum, then gets the 'Confid', 'Range_H',
        'Count' values with highest confidence sum for 2 sources.
        Step2: Compares the highest 'confidence sum / signal update frequency'
        for two sources, and get which source should be used, then the selected
        confidence sum and 'Range_H' value of that source are get.
        Step3: If the confidence sum is higher than a threshold, then the
        trailer length is equal to the selected 'Range_H' value. The reason why
        use the 'Range_H' value as the trailer length is longer trailer is
        safer for bounding box estimation. The confidence is calculated as:
        (confidence_sum / corresponding counter). If the confidence sum is
        lower than the threshold, use the default value for trailer length and
        confidence.
        """
        max_confid_sum_sr = sort_large(self.len_range_df_sr,
                                       "Confid",
                                       "Confid")
        trailer_len_table_sr = sort_large(self.len_range_df_sr,
                                          "Range_H",
                                          "Confid")
        count_sr = sort_large(self.len_range_df_sr,
                              "Count",
                              "Confid")

        max_confid_sum_cv = sort_large(self.len_range_df_cv,
                                       "Confid",
                                       "Confid"
                                       )
        trailer_len_table_cv = sort_large(self.len_range_df_cv,
                                          "Range_H",
                                          "Confid"
                                          )
        count_cv = sort_large(self.len_range_df_cv,
                              "Count",
                              "Confid")

        max_index = np.argmax([max_confid_sum_sr * self.trailer_len_cycle_sr,
                               max_confid_sum_cv * self.trailer_len_cycle_cv])
        max_confid_sum = [max_confid_sum_sr, max_confid_sum_cv][max_index]
        trailer_len_table = ([trailer_len_table_sr, trailer_len_table_cv]
                             [max_index])

        if max_confid_sum > self.confid_sum_threshold:
            self.trailer_len = trailer_len_table
            max_count = [count_sr, count_cv][max_index]
            self.trailer_len_confid = max_confid_sum / max_count
        else:
            self.trailer_len = self.trailer_len_default
            self.trailer_len_confid = self.trailer_len_confid_default


class Ego_State:
    """This class is the main class of Ego_State module.
    This class strores all the parameters as the parameter pool, and 'update'
    method is the core method that is called by ROS node.
    Also, there are some ego states calculation are easy to get, so they are
    not defined as a Class outside, such as 'get_eng2whl_ratio',
    'get_tractor_yawrate', 'get_max_acceleration', and 'get_ecoroll_acc'.
    Moreover, some init method are defined and called in this class to
    initialize some paramters, such as '_weight_data_init', '_speed_data_init',
    '_artic_angle_init', '_trailer_len_init'.
    'parse_mat' method is used for offline test, not for online ROS.
    """

    def __init__(self):
        # 0 for df, 1 for dict
        self.unit_conv = UnitConverter()
        self.config = Config()
        # 输入已知参数
        self.hitch2rear_axle = 0.  # tractor wheel base
        self.trailer_wheelbase = 8.1  # trailer wheel base
        self.artic_angle_vd_ini = 0.
        self.artic_angle_vd = self.artic_angle_vd_ini
        self.wheel_angle = 0.
        self.forward_drive_distance = 0.
        self.time_temp = 0.

        self.trctr_yr_maf_window = deque(maxlen=20)
        self.tractor_yawrate = 0.  # from VDC2

        self.artic_angle_ini_wb = 0.
        self.artic_angle_calc_wb = self.artic_angle_ini_wb
        self.trailer_yawrate_wb = 0.
        self.trailer_wheelbase_wb = 8.1  # trailer estimated wheel base
        # trailer estimated wheel base finally be saved
        self.trailer_wheelbase_wb_save = self.trailer_wheelbase_wb
        self.trailer_wheelbase_wb_prev = self.trailer_wheelbase_wb
        self.wb_stable_time = 0.
        self.trailer_wheelbase_wb_min = 6.
        self.trailer_wheelbase_wb_max = 10.

        # self.speed_arr = np.zeros((4, 3))
        self.hrw_speed_fl = 0.  # from HRW
        self.hrw_speed_fr = 0.  # from HRW
        self.tsmo_rpm = 0.  # from ETC1
        self.tsmo_speed = 0.
        self.tsmi_rpm = 0.  # from ETC1
        self.tsmi_rpm_prev = 0.
        self.tsmi_speed = 0.
        self.tach_speed = 0.  # from TCO1
        self.gear_ratio = 65.535  # from ETC2
        self.gear_ratio_prev = 65.535
        self.current_gear = 0
        self.current_gear_prev = 0
        self.veh_speed = 0.  # kph
        self.veh_speed_prev = 0.
        self.shift_process = 0
        self.speed_valid_range = {
            "HRW": (2., 250.),
            "TSMO": (0.05, 250.),
            "TSMI": (0., 4000.),
            "TACH": (0., 250.)
        }

        self.priority_table = {
            "max_speed": [1e1000, 3., 1.],
            "min_speed": [3., 1., -1e1000],
            "priority_list": [[1, 2, 3, 4], [3, 2, 1, 4], [3, 1, 2, 4]]
        }

        self.weight_amt = 0.  # from cvw_amt
        self.weight_ebs = 0.  # from cvw_ebs
        self.veh_weight = 18000.
        self.veh_weight_prev = self.veh_weight
        self.weight_valid_range = {
            "CVW_AMT": (8800., 50000.),
            "CVW_EBS": (8800., 50000.)
        }

        self.weight_priority_table = {"Priority": [1, 2]}
        
        self.weight_priority_table_dict = {"CVW_AMT": 1, "CVW_EBS": 2}

        # x: speed(kph), y: deceleration(kph/s) by test result
        self.ecoroll_acc_table = [[3, 5, 10, 20, 30, 40, 50],
                                  [
                                      -0.3, -0.27, -0.34, -0.39, -0.43, -0.54,
                                      -0.6
                                  ]]

        self.reference_torque = 3312  # from EC1, default value based on blf
        self.friction_torque = 4  # from EEC3, default value based on blf
        self.max_available_torque = 40.  # from EEC2, default val based on blf

        self.eng2whl_ratio = 0.  # from Ego_State
        self.eng2whl_ratio_prev = self.eng2whl_ratio
        self.eng2whl_maf_window = deque(maxlen=20)
        self.gear_ratio_uplimit = 60.
        self.tsmi_rpm_uplimit = 8000.  # gear shift period, invalid value 8191.
        # vehicle launch or stop period less than the following number
        self.veh_speed_lwlimit = 0.5
        # wheel angle uplimit of straight drive
        self.wheel_angle_strght_uplimit = 0.3
        # straight driving at least the distance as following number
        self.forward_drive_distance_lwlimit = 30.

        self.angle_priority_table = {"SR": 1, "VD": 2, "CV": 3}
        self.artic_angle_sr = 0.  # from TRD (VCOM.dbc)
        self.artic_angle_cv = 0.
        self.artic_angle_prev = 0.  # rad
        self.artic_angle = self.artic_angle_prev
        self.artic_angle_max = math.pi
        self.artic_angle_min = -math.pi
        self.artic_angle_confid_sr = 0.05  # from TRD (VCOM.dbc)
        self.artic_angle_confid_cv = 0.05
        self.artic_angle_confid_vd = 0.9
        self.artic_angle_confid_vd_lwlimit = 0.5
        self.artic_angle_confid_vd_decel = -0.000001  # decrease 0.12 per min
        self.artic_angle_confid = 0.

        self.trailer_len = 0.
        self.trailer_len_confid = 0.
        self.len_range_table = {
            "Range_L": np.arange(5, 17.5, 0.5),
            "Range_H": np.arange(5.5, 18, 0.5)
        }
        self.trailer_len_sr = 0.  # from TRD (VCOM.dbc)
        self.trailer_len_confid_sr = 0.  # from TRD (VCOM.dbc)
        self.trailer_len_cv = 0.
        self.trailer_len_confid_cv = 0.
        self.len_range_df_sr = pd.DataFrame(self.len_range_table)
        self.len_range_df_sr["Confid"] = 0.
        self.len_range_df_sr["Count"] = 0.
        self.len_range_df_cv = pd.DataFrame(self.len_range_table)
        self.len_range_df_cv["Confid"] = 0.
        self.len_range_df_cv["Count"] = 0.
        self.trailer_len_default = 13.
        self.trailer_len_confid_default = 0.5
        #  The following is for can msg receiving from CAN node and forward to
        #  others self.xbr_deceleration_limit = 0
        self.a = speed_source
        if speed_source == 0:
            self._speed_data_init()
        else:
            self._speed_data_init_dict()

        if weight_source == 0:
            self._weight_data_init()
        else:
            self._weight_data_init_dict()
        
        if artic_angle_source == 0:
            self._artic_angle_init()
        else:
            self._artic_angle_init_dict()

        self._trailer_len_init()

        self.veh_speed_downgrade = 0
        self.veh_weight_downgrade = 0
        self.artic_angle_downgrade = 0

# =============================================================================

    def update(self):
        """This method is the main method of this whole Ego_State module.
        The most core method in the "EgoState" module is responsible for
        instantiating several other sub-functionality classes. During
        instantiation, it passes the parameters from the parameter pool of the
        "EgoState" class to these sub-functionality classes. It then calls the
        main computation function in each sub-functionality class. Once the
        computation is completed, it updates the parameters and results back
        to the "EgoState" class for parameter updates. This way, in the next
        round of calling this method, the latest parameters can be used to
        initialize the other sub-functionality classes."""
#===========================================================================
# SPEED
        self.vs = VehSpd(self)
        if speed_source == 0:
            self.vs.get_veh_speed()
        else:
            self.vs.get_veh_speed_dict()
        self.gear_ratio = self.vs.gear_ratio
        self.gear_ratio_prev = self.vs.gear_ratio_prev
        self.current_gear = self.vs.current_gear
        self.current_gear_prev = self.vs.current_gear_prev
        self.veh_speed = self.vs.veh_speed
        self.tsmo_speed = self.vs.tsmo_speed
        self.tsmi_speed = self.vs.tsmi_speed
        self.tsmi_rpm = self.vs.tsmi_rpm
        self.tsmi_rpm_prev = self.vs.tsmi_rpm_prev
        self.veh_speed_prev = self.vs.veh_speed_prev
        if speed_source == 0:
            self.speed_df = self.vs.speed_df
        else:
            self.speed_dict = self.vs.speed_dict
        self.veh_speed_downgrade = self.vs.veh_speed_downgrade
#=========================================================================
# WEIGHT
        self.vw = VehWght(self)
        if weight_source == 0:
            self.vw.get_veh_weight()
        else:
            self.vw.get_veh_weight_dict()
        self.veh_weight = self.vw.veh_weight
        self.veh_weight_prev = self.vw.veh_weight_prev
        if weight_source == 0:
            self.weight_df = self.vw.weight_df
        else:
            self.weight_dict = self.vw.weight_dict
        self.veh_weight_downgrade = self.vw.veh_weight_downgrade
#==========================================================================
# ANGLE
        self.aa = ArtAng(self)
        self.aa.calc_artic_angle()
        # angle = self.aa.calc_artic_angle()
        # plotter.plot_articulation_angle(ego_state.time, angle)
        # plotter.plot_articulation_angle_meas(ego_state.time,
        #                                      ego_state.new_mat[
        #                                       "Yaw_Rate_Estimated"])
        self.artic_angle_vd = self.aa.artic_angle_vd
        self.artic_angle_confid_vd = self.aa.artic_angle_confid_vd
        self.forward_drive_distance = self.aa.forward_drive_distance
# fuse
        if weight_source == 0:
            self.aa.fuse_artic_angle()
            self.artic_angle_df = self.aa.artic_angle_df
        else:
            self.aa.fuse_artic_angle_dict()
            self.artic_angle_dict = self.aa.artic_angle_dict
        self.artic_angle = self.aa.artic_angle
        self.artic_angle_confid = self.aa.artic_angle_confid
        self.artic_angle_prev = self.aa.artic_angle_prev
        self.artic_angle_downgrade = self.aa.artic_angle_downgrade
#==========================================================================
# WHEEL_BASE
        self.wb = WheelBase(self)
        if self.artic_angle_confid_sr > 0.9:
            self.wb.cal_wheelbase_iterated()
        # trailer_wheelbase_array = self.wb.cal_wheelbase_iterated()
        # plotter.plot_articulation_angle(ego_state.time, trailer_wheelbase_array)
        # plotter.show_plot()
        self.artic_angle_calc_wb = self.wb.artic_angle_calc_wb  # need 2 return
        self.trailer_yawrate_wb = self.wb.trailer_yawrate_wb  # need 2 return
        self.trailer_wheelbase_wb = self.wb.trailer_wheelbase_wb  # need 2 return
        self.trailer_wheelbase_wb_save = self.wb.trailer_wheelbase_wb_save  # need 2 return
        self.trailer_wheelbase_wb_prev = self.wb.trailer_wheelbase_wb_prev  # need 2 return
        self.wb_stable_time = self.wb.wb_stable_time  # need 2 return
#===========================================================================
# TRAILER_LENGTH
        if trailer_length_source == 0:
            self.tl = TrlrLen(self)
            self.tl.determine_trailer_length()
            self.trailer_len = self.tl.trailer_len
            self.trailer_len_confid = self.tl.trailer_len_confid
        else:
            self.trailer_len = self.trailer_len_default
            self.trailer_len_confid = self.trailer_len_confid_default
# =============================================================================
#  legacy common function for offline test

    def parse_mat(self):
        """This method is used to load a recorded mat file from ADOPT3 test.
        The test file is in MF4 format, and from the test file, some interested
        signals are picked and exported as a mat file.
        This method parses the mat file, clears the mat data, and creates a
        dict type of data."""
        mat = loadmat("ArticulationAngleTest1019.mat")
        self.new_mat = mat.copy()
        keys_to_delete = ["__globals__", "__header__", "__version__"]
        for key in keys_to_delete:
            if key in self.new_mat:
                del self.new_mat[key]
        self.time = self.new_mat["Yaw_Rate"][:, 0]
        for key, values in self.new_mat.items():
            self.new_mat[key] = values[:, 1]

        #  below is manually generated for test, but wheel_angle from VDC2
        #  should replace
        self.wheel_angle = np.random.uniform(0.31, 0.35, size=self.time.size)
        self.wheel_angle[000 * 100:000 * 100] = 0.2

# =============================================================================
# Calculate vehicle weight by pandas

    def _weight_data_init(self):
        """
        This method creates a pd.DataFrame which stores the weight data for
        arbitration. The DataFrame is 2*5, 2 rows are ["CVW_AMT", "CVW_EBS"],
        and 5 columns are ["Weight", "Validity", "Priority", "Min", "Max"].
        """
        index_list = ["CVW_AMT", "CVW_EBS"]
        colum_list = ["Weight", "Validity"]
        data_shape = (len(index_list), len(colum_list))
        weight_df = pd.DataFrame(np.zeros(data_shape),
                                 index=index_list,
                                 columns=colum_list)
        weight_prio = pd.DataFrame(self.weight_priority_table,
                                   index=["CVW_AMT", "CVW_EBS"])
        weight_df_add = pd.DataFrame(self.weight_valid_range,
                                     index=["Min", "Max"]).T
        self.weight_df = pd.concat([weight_df, weight_prio, weight_df_add],
                                   axis=1)

    def _weight_data_init_dict(self):
        """
        This method creates a pd.DataFrame which stores the weight data for
        arbitration. The DataFrame is 2*5, 2 rows are ["CVW_AMT", "CVW_EBS"],
        and 5 columns are ["Weight", "Validity", "Priority", "Min", "Max"].
        """

        self.weight_dict = {}
        index_list = ["CVW_AMT", "CVW_EBS"]
        colum_list = ["Weight", "Validity", "Priority", "Min", "Max"]
        for i in index_list:
            self.weight_dict[i] = {}
            for j in colum_list:
                if j == "Priority":
                    self.weight_dict[i][j] = self.weight_priority_table_dict[i]
                elif j == "Min":
                    self.weight_dict[i][j] = self.weight_valid_range[i][0]
                elif j == "Max":
                    self.weight_dict[i][j] = self.weight_valid_range[i][1]
                else:
                    self.weight_dict[i][j] = 0
        
# =============================================================================
# Calculate vehicle speed by pandas

    def _speed_data_init(self):
        """
        This method creates a pd.DataFrame which stores the speed data for
        arbitration. The DataFrame is 4*5, 4 rows are ["HRW", "TSMO", "TSMI",
                                                       "TACH"],
        and 5 columns are ["Speed", "Priority", "Validity", "Min", "Max"].
        """
        index_list = ["HRW", "TSMO", "TSMI", "TACH"]
        colum_list = ["Speed", "Priority", "Validity"]
        data_shape = (len(index_list), len(colum_list))
        speed_df = pd.DataFrame(np.zeros(data_shape),
                                index=index_list,
                                columns=colum_list)
        speed_df["Validity"] = False
        speed_df_add = pd.DataFrame(self.speed_valid_range,
                                    index=["Min", "Max"]).T
        self.speed_df = pd.concat([speed_df, speed_df_add], axis=1)
        
    def _speed_data_init_dict(self):
        """
        This method creates a pd.DataFrame which stores the speed data for
        arbitration. The DataFrame is 4*5, 4 rows are ["HRW", "TSMO", "TSMI",
                                                       "TACH"],
        and 5 columns are ["Speed", "Priority", "Validity", "Min", "Max"].
        """
        self.speed_dict = {}
        index_list = ["HRW", "TSMO", "TSMI", "TACH"]
        colum_list = ["Speed", "Priority", "Validity", "Min", "Max"]
        for i in index_list:
            self.speed_dict[i] = {}
            for j in colum_list:
                if j == "Validity":
                    self.speed_dict[i][j] = False
                elif j == "Min":
                    self.speed_dict[i][j] = self.speed_valid_range[i][0]
                elif j == "Max":
                    self.speed_dict[i][j] = self.speed_valid_range[i][1]
                else:
                    self.speed_dict[i][j] = 0
  

# =============================================================================
# Articulation Angle Fusion

    def _artic_angle_init(self):
        """
        This method creates a pd.DataFrame which stores the angle data for
        arbitration. The DataFrame is 3*4, 3 rows are ["SR", "VD", "CV"],
        and 4 columns are ["Angle", "Priority", "Confid", "Validity"].
        """
        index_list = ["SR", "VD", "CV"]
        colum_list = ["Angle", "Priority", "Confid", "Validity"]
        data_shape = (len(index_list), len(colum_list))
        self.artic_angle_df = pd.DataFrame(np.zeros(data_shape),
                                           index=index_list,
                                           columns=colum_list)
        self.artic_angle_df["Priority"] = [
            self.angle_priority_table["SR"], self.angle_priority_table["VD"],
            self.angle_priority_table["CV"]
        ]
        self.artic_angle_df["Validity"] = False
        
    def _artic_angle_init_dict(self):
        """
        This method creates a pd.DataFrame which stores the angle data for
        arbitration. The DataFrame is 3*4, 3 rows are ["SR", "VD", "CV"],
        and 4 columns are ["Angle", "Priority", "Confid", "Validity"].
        """
        index_list = ["SR", "VD", "CV"]
        colum_list = ["Angle", "Priority", "Confid", "Validity"]
        self.artic_angle_dict = {}
        for i in index_list:
            self.artic_angle_dict[i] = {}
            for j in colum_list:
                if j == "Priority":
                    self.artic_angle_dict[i][j] = self.angle_priority_table[i]
                elif j == "Validity":
                    self.artic_angle_dict[i][j] = False
                else:
                    self.artic_angle_dict[i][j] = 0.0

# =============================================================================
# Calculate eng2whl_ratio

    def get_eng2whl_ratio(self):
        """This method calculates the input shaft to wheel ratio.
        This ratio is (GearRatio * DiffRatio / Radius).
        The calculation is based on the following equation:
        VehSpd(kph) = InputShaftSpeed(rpm) / 60 / x * 2 * pi * 3.6
        When the transmission is gear shifting, the transmission input shaft
        speed value is invalid or when vehicle speed is very small, the
        calculation of eng2whl_ratio cannot be proceeded, the eng2whl_ratio
        should be kept as last value.
        Otherwise, the calculation should be done by the upper equation, and
        a moving average filter is applied.
        """
        if ((self.shift_process == 1)
                or (abs(self.veh_speed) < self.veh_speed_lwlimit)
                or (self.current_gear == 0)):
            self.eng2whl_ratio = self.eng2whl_ratio_prev
        else:
            self.eng2whl_ratio = (self.tsmi_rpm / abs(self.veh_speed) *
                                  self.unit_conv.rpm2kph_wo_radius)
            self.eng2whl_ratio = filter_move_average(self.eng2whl_ratio,
                                                     self.eng2whl_maf_window)
            self.eng2whl_ratio_prev = self.eng2whl_ratio

# =============================================================================
# Calculate tractor yawrate

    def get_tractor_yawrate(self):
        if self.veh_speed == 0.0:
            self.tractor_yawrate = 0.0
        self.tractor_yawrate = filter_move_average(self.tractor_yawrate,
                                                   self.trctr_yr_maf_window)

# =============================================================================
# Calculate max_acc

    def get_max_acceleration(self):
        """This method calculates the max available vehicle accelartion based
        on the max engine torque.
        unit: m/s2"""
        # outshaft_rpm2veh_spd = 5.1765
        if self.current_gear != 0:
            self.max_acc = max(
                0.,
                ((self.max_available_torque - self.friction_torque) / 100 *
                self.reference_torque * self.eng2whl_ratio + self.road_resistance)
                / self.veh_weight)
        else:
            self.max_acc = 0.
# =============================================================================

# Calculate eco_roll acceleration and resistance force

    def get_ecoroll_acc(self):
        """This method calculates the ecoroll deceleration and road resistance.
        The table of the relationship between ecoroll deceleration and vehicle
        speed is get by a pre tested result from K7 by Xuyuan, and the road
        resistance is get by the ecoroll deceleration and vehicle weight.
        """
        self.ecoroll_acc_array = np.array(self.ecoroll_acc_table)
        self.a_ecoroll = np.interp(
            abs(self.veh_speed), self.ecoroll_acc_array[0],
            self.ecoroll_acc_array[1] * self.unit_conv.kph2mps)
        self.road_resistance = self.a_ecoroll * self.veh_weight  # negative

# =============================================================================
# Trailer Length Fusion

    def _trailer_len_init(self):
        """
        This method defines 'min_detect_angle' as the minimum art angle that
        the radar and cv can detect the trailer length, 'trailer_len_cycle_sr'
        and 'trailer_len_cycle_sr' as the source update cycle time,
        'confid_sum_threshold' as the min value of the sum of confidence that
        the trailer length range is big enough to be used as result.
        """
        # self.trailer_len_sr_arr = np.random.uniform(5, 17.5, size=1000)
        # self.trailer_len_cv_arr = np.random.uniform(5, 17.5, size=2000)
        self.min_detect_angle = math.pi / 20.
        self.trailer_len_cycle_sr = 0.06  # from VCOM dbc checked
        self.trailer_len_cycle_cv = 0.05  # assumed
        self.confid_sum_threshold = 2.

    def _len_update(self, trailer_len, trailer_len_confid, len_df):
        """
        This is the common method that updates the trailer length DataFrame.
        If a new trailer length value comes, first find which length range it
        locates, and at this moment, the articulation angle should be big
        enough for a valid detection. If the upper condition fulfills, the
        DataFrame should be updated, the way to update the DataFrame is to
        add the incomming confidence to the corresponding range row, and count
        once.
        """

        valid_cond = ((trailer_len > self.len_range_table["Range_L"].min())
                      & (trailer_len <= self.len_range_table["Range_H"].max())
                      & (abs(self.artic_angle) > self.min_detect_angle))

        if valid_cond:
            select = ((trailer_len > len_df["Range_L"]) &
                      (trailer_len <= len_df["Range_H"]))
            len_df.loc[select, "Confid"] += trailer_len_confid
            len_df.loc[select, "Count"] += 1

    def len_sr_update(self):
        """
        This method is a callback function that be called when the ROS node
        reiceives a trailer length from side radar.
        """
        self._len_update(self.trailer_len_sr, self.trailer_len_confid_sr,
                         self.len_range_df_sr)

    def len_cv_update(self):
        """
        This method is a callback function that be called when the ROS node
        reiceives a trailer length from cv.
        """
        self._len_update(self.trailer_len_cv, self.trailer_len_confid_cv,
                         self.len_range_df_cv)

speed_source = 1
weight_source = 1
artic_angle_source = 1
trailer_length_source = 1

if __name__ == "__main__":

    ego_state = Ego_State()
    ego_state.hrw_speed_fl = 5.
    ego_state.hrw_speed_fr = 4.
    ego_state.tsmo_rpm = 10.
    ego_state.tsmi_rpm = 1000.
    ego_state.tach_speed = 2.

    ego_state.weight_amt = 8000
    ego_state.weight_ebs = 8000
    
    ego_state.artic_angle_vd =  1.3
    ego_state.artic_angle_confid_vd = 0.1
    ego_state.artic_angle_sr =  0.1
    ego_state.artic_angle_confid_sr =  0.1
    ego_state.artic_angle_cv =  1.4
    ego_state.artic_angle_confid_cv =  0.1

    # plotter = Plot()
    # ego_state.parse_mat()
    # ============================================================================
    # Calculate Articulation Angle
    
    # angle = ego_state.calc_artic_angle()
    # plotter.plot_articulation_angle(ego_state.time, angle)
    # plotter.plot_articulation_angle_meas(ego_state.time,
    #                                       ego_state.new_mat[
    #                                           "Yaw_Rate_Estimated"])
    # ============================================================================
    # Calculate Wheel Base
    
    # trailer_wheelbase_array = ego_state.cal_wheelbase_iterated()
    # plotter.plot_articulation_angle(ego_state.time, trailer_wheelbase_array)
    # plotter.show_plot()
    # ============================================================================
    # Calculate Vehicle Speed


    ego_state.get_eng2whl_ratio()
    vs = VehSpd(ego_state)
    vw = VehWght(ego_state)
    aa = ArtAng(ego_state)
    prof = line_profiler.LineProfiler(aa.fuse_artic_angle) ############
    prof.enable()
    
    if speed_source == 1:
        vs.get_veh_speed_dict()
        vs.get_veh_speed_dict()
        speed_dict = vs.speed_dict
        speed = vs.veh_speed
    else:
        vs.get_veh_speed()
        vs.get_veh_speed()
        speed_df = vs.speed_df
        speed = vs.veh_speed
        
    if weight_source == 1:
        vw.get_veh_weight_dict()
        vw.get_veh_weight_dict()
        weight_dict = vw.weight_dict
        weight = vw.veh_weight
    else:
        vw.get_veh_weight()
        vw.get_veh_weight()
        weight_df = vw.weight_df
        weight = vw.veh_weight
        
    if artic_angle_source == 1:
        aa.fuse_artic_angle_dict()
        aa.fuse_artic_angle_dict()
        angle_dict = aa.artic_angle_dict
        angle = aa.artic_angle
    else:
        aa.fuse_artic_angle()
        aa.fuse_artic_angle()
        angle_df = aa.artic_angle_df
        angle = aa.artic_angle
        
    prof.disable()
    prof.print_stats(sys.stdout)
    # ego_state.update()
    # ego_state.get_veh_weight()
    # ============================================================================
    # Fuse artic_angle
    # ego_state.fuse_artic_angle()
