#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# By yanglitao
import numpy as np
import math
class Imm:
    def __init__(self, models, model_trans, P_trans, U_prob):
        """初始化IMM滤波器"""
        self.models = models              # 模型列表
        self.model_trans = model_trans    # 模型状态转移矩阵
        self.P_trans = P_trans            # 模型转移概率矩阵
        self.U_prob = U_prob              # 初始模型概率
        self.mode_cnt = len(models)       # 模型数量
        self.dim = models[0].A.shape[0]   # 状态维度
    
    def filt(self, Z):
        """执行IMM滤波迭代"""
        # 步骤1：输入交互
        u = np.dot(self.P_trans.T, self.U_prob)  # 非归一化后验概率
        mu = np.zeros(self.P_trans.shape)
        
        # 计算模型转移概率
        for i in range(self.mode_cnt):
            for j in range(self.mode_cnt):
                mu[i, j] = self.P_trans[i, j] * self.U_prob[i, 0] / u[j, 0]
        
        # 计算混合状态和协方差
       # 计算混合状态估计
        X_mix = [np.zeros(model.X.shape) for model in self.models]
        for j in range(self.mode_cnt):
            for i in range(self.mode_cnt):
                X_mix[j] += np.dot(self.model_trans[j][i], self.models[i].X) * mu[i, j]
                # print(X_mix[j])
                # print(self.model_trans[j][i])
                # print(self.models[i].X)
                # print(mu[i, j])
                # print('------------------')
        # 计算混合协方差估计
        P_mix = [np.zeros(model.P.shape) for model in self.models]
        for j in range(self.mode_cnt):
            for i in range(self.mode_cnt):
                P = self.models[i].P + np.dot(
                    (self.models[i].X - X_mix[i]), (self.models[i].X - X_mix[i]).T
                )
                P_mix[j] += mu[i, j] * np.dot(
                    np.dot(self.model_trans[j][i], P), self.model_trans[j][i].T
                )
        
        # 步骤2：各模型滤波
        for j in range(self.mode_cnt):
            self.models[j].X = X_mix[j]
            self.models[j].P = P_mix[j]
            self.models[j].filt(Z)
        # 步骤3：更新模型概率（对数域计算，避免数值下溢，并保持稳定）
        log_weights = np.empty((self.mode_cnt, 1))
        for j in range(self.mode_cnt):
            mode = self.models[j]
            D = Z - mode.H @ mode.X_pre
            S = mode.H @ mode.P_pre @ mode.H.T + mode.R

            sign, logdet = np.linalg.slogdet(2 * math.pi * S)
            if sign <= 0:
                # 异常情况下退回到常规行列式以避免 nan
                logdet = np.log(np.finfo(float).eps)

            quad = float(D.T @ np.linalg.solve(S, D))
            log_weights[j, 0] = -0.5 * (logdet + quad) + np.log(u[j, 0])

        # 归一化
        max_log = np.max(log_weights)
        exp_weights = np.exp(log_weights - max_log)
        self.U_prob = exp_weights / np.sum(exp_weights)

        # # 步骤3：更新模型概率
        # for j in range(self.mode_cnt):
        #     mode = self.models[j]
        #     D = Z - np.dot(mode.H, mode.X_pre)
        #     S = np.dot(np.dot(mode.H, mode.P_pre), mode.H.T) + mode.R

        #     Lambda = (np.linalg.det(2 * math.pi * S)) ** (-0.5) * np.exp(
        #         -0.5 * np.dot(np.dot(D.T, np.linalg.inv(S)), D)
        #     )

        #     self.U_prob[j, 0] = Lambda * u[j, 0]
        # self.U_prob = self.U_prob / np.sum(self.U_prob)

        return self.U_prob
    
    def update_model_set(self, new_models, new_P_trans, new_U_prob):
        """更新模型集、转移概率和初始概率"""
        self.models = new_models
        self.P_trans = new_P_trans
        self.U_prob = new_U_prob
        self.mode_cnt = len(new_models)
        self.dim = new_models[0].A.shape[0]
    
    # def update_model_params(self, model_idx, params):
    #     """更新指定模型的参数"""
    #     if 0 <= model_idx < len(self.models):
    #         self.models[model_idx].A = params['A']
    #         self.models[model_idx].H = params['H']
    #         self.models[model_idx].Q = params['Q']
    #     else:
    #         raise IndexError("模型索引超出范围")