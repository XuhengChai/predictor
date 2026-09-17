#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# By yanglitao

import math
import numpy as np


class KalmanFilter:

    def __init__(self, A, H, Q):
        self.A = A # 状态转移矩阵
        self.B = np.eye(A.shape[0])#初始化控制矩阵为单位矩阵
        self.H = H # 观测矩阵
        self.Q = Q * np.eye(A.shape[0]) # 过程噪声协方差矩阵
        self.R = 0.01 * np.eye(H.shape[0]) # 初始化观察噪声协方差矩阵为单位矩阵 为传感器噪声，为固定值

        self.U = np.zeros((self.B.shape[1], 1))  # 初始控制输入
        self.X = np.zeros((A.shape[0], 1)) # 初始状态
        self.X_pre = self.X # 预测状态
        self.P = np.zeros(A.shape) # 误差协方差矩阵
        self.P_pre = self.P # 预测协方差

    def filt(self, Z): 
        self.__predict(Z)
        self.__update(Z)
        return self.X

    def __predict(self, Z): # 预测
        self.X_pre = np.dot(self.A, self.X) + np.dot(self.B, self.U)
        self.P_pre = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
       

    def __update(self, update_z): # 更新
        K = np.dot( # 计算卡尔曼增益
            np.dot(self.P_pre, self.H.T),
            np.linalg.inv(np.dot(np.dot(self.H, self.P_pre), self.H.T) + self.R),
        )
        self.X = self.X_pre + np.dot(K, update_z - np.dot(self.H, self.X_pre)) # 更新状态
        self.P = self.P_pre - np.dot(np.dot(K, self.H), self.P_pre) # 更新协方差
