import os
import sys
from os.path import join as pjoin
from datetime import datetime

import time
import shutil
import pickle
import numpy as np
import torch

# from torch.utils.data import DataLoader
from torch_geometric.data import DataLoader

from torch_geometric.data import Data
import onnxruntime as ort
import json
import pandas as pd
from enum import IntEnum
ort.set_default_logger_severity(3)

class GraphData(Data):
    """
    override key `cluster` indicating which polyline_id is for the vector
    """
    def __inc__(self, key, value):
        if key == 'edge_index':
            return self.x.size(0)
        elif key == 'cluster':
            return int(self.cluster.max().item()) + 1
        else:
            return 0

def to_numpy(t):
    return t.detach().cpu().numpy() if isinstance(t, torch.Tensor) else np.asarray(t)

class EAgentType(IntEnum):
    PEDESTRIAN = 0
    BICYCLE = 1
    CYCLIST = 1
    MOTORCYCLIST = 2

class TNTOnlineInfer:
    def __init__(self, onnx_tnt=None, onnx_mdn=None, device='cpu'):
        self.k = 6  # number of trajectories to select
        self.m = 50  # number of candidate trajectories
        self.horizon = 30  # prediction horizon (number of time steps)
        self.device = device
        self.session_tnt = None
        self.session_mdn = None
        if onnx_tnt:
            try:
                self.session_tnt = ort.InferenceSession(onnx_tnt, providers=["CPUExecutionProvider"])
                self.input_name_tnt = [i.name for i in self.session_tnt.get_inputs()]
            except Exception as e:
                raise RuntimeError(f"Failed to create ONNX session for {onnx_tnt}: {e}")
        if onnx_mdn:
            try:
                self.session_mdn = ort.InferenceSession(onnx_mdn, providers=["CPUExecutionProvider"])
                self.input_name_mdn = [i.name for i in self.session_mdn.get_inputs()]
            except Exception as e:
                raise RuntimeError(f"Failed to create ONNX session for {onnx_mdn}: {e}")

    def infer_with_mdn(self, data):
        # start = time.perf_counter()
        forecasted_trajectories, pred_y, score_y = self.infer(data)
        # end = time.perf_counter()
        # print(f"执行时间: {end - start:.6f} 秒")
        if pred_y.shape[1] != self.k:
            raise ValueError(f"Expected {self.k} trajectories from TNT inference, got {pred_y.shape[1]}")
        batch_size = len(forecasted_trajectories)

        ids = data.x[:, 9]
        self_array = data.x[ids == 0] #batch_size, feat_dim
        mask = self_array.any(axis=1)
        mask[0] =True
        self_array = self_array[mask][:,:2]
        self_array = self_array.cpu().view(batch_size, -1, self_array.shape[1]).numpy() # batch_size, feat_dim
        origs = data.orig.numpy()
        rots = data.rot.numpy()
        angles = np.zeros(batch_size, dtype=np.float32)
        for batch_id in range(batch_size):
            self_array[batch_id] = self.convert_coord(self_array[batch_id], origs[batch_id], rots[batch_id])
            origs[batch_id] = self_array[batch_id][-1].copy()
            dxy = self_array[batch_id][0] - self_array[batch_id][-1]
            angles[batch_id] = -np.arctan2(dxy[1], dxy[0])

        # dummy_x_hist = torch.randn(1, 30, 4) # 示例输入
        # dummy_x_cand = torch.randn(1, 6, 30, 4)  # 示例输入
        # dummy_wi = torch.randn(1, 6)  # 示例输入
        # dummy_cls_feature = torch.randn(1, 6, 23)  # 示例输入
        is_rotate = True
        seq_ids = data.seq_id.numpy()
        if self.session_mdn is not None:
            # Run ONNX inference for MDN
            cnt = 0
            # concat the candidate trajectories and scores into the input for MDN
            dummy_x_hist = np.zeros((batch_size, 30, 4), dtype=np.float32)
            dummy_x_cand = np.zeros((batch_size, self.k, 30, 4), dtype=np.float32)
            dummy_wi = np.zeros((batch_size, self.k), dtype=np.float32)
            dummy_cls_feature = np.zeros((batch_size, self.k, 23), dtype=np.float32)
            for seq_id, cand_traj in forecasted_trajectories.items():
                # print(f"Seq ID: {key}")

                hist_traj = self_array[cnt].copy()  # batch_size, 30, feat_dim
                origin = origs[cnt]
                angle = angles[cnt]
                hist_traj -= origin
                if is_rotate:
                    hist_traj = self.rotate_xy_numpy(hist_traj, angle)
                hist_traj = self.cal_dxy(hist_traj)

                if len(cand_traj) != self.k:
                    print(f"Warning: Expected {self.k} trajectories for seq_id {seq_id}, but got {len(cand_traj)}")
                    return None
                for key, val in cand_traj.items():
                    # print(f"  Score: {key}, Trajectory shape: {val.shape}")
                    cand_traj[key] -= origin
                    if is_rotate:
                        cand_traj[key] = self.rotate_xy_numpy(cand_traj[key], angle)
                    cand_traj[key] = self.cal_dxy(cand_traj[key])

                keys = list(cand_traj.keys())
                trajs = list(cand_traj.values())
                cand_key_array = np.array(keys, dtype=float)  # .reshape(-1, 1)
                cand_key_array = cand_key_array / np.sum(cand_key_array)
                cand_traj_array = np.stack(trajs, axis=0)

                cand_feat = self.calculate_enhanced_candidate_features(hist_traj, cand_traj_array)
                cls_id = data.class_id[cnt].item() if hasattr(data, 'class_id') else 1
                is_pedestrian = (cls_id == EAgentType.PEDESTRIAN)
                is_motorcycle_tricycle = (cls_id == EAgentType.MOTORCYCLIST)
                is_bicycle = (cls_id == EAgentType.BICYCLE)
                features = np.full((len(cand_key_array), 3), [is_pedestrian, is_motorcycle_tricycle, is_bicycle])
                cls_frames = np.concatenate((features, cand_feat), axis=1)

                dummy_x_hist[cnt] = hist_traj
                dummy_wi[cnt] = cand_key_array
                dummy_x_cand[cnt] = cand_traj_array
                dummy_cls_feature[cnt] = cls_frames
                cnt += 1
            inputs = {
                'x_hist': dummy_x_hist,
                'x_cand': dummy_x_cand,
                'cls_feature': dummy_cls_feature,
                'wi': dummy_wi
            }

            feed = {name: inputs[name] for name in self.input_name_mdn if name in inputs}
            mdn_outs = self.session_mdn.run(None, feed)
            pi, mu, sigma, logits = mdn_outs

            # 将 torch 代码转换为 numpy
            top1 = np.argmax(pi, axis=-1)  # [B]

            # 计算 ADE/FDE 的 min@K 与 Top-1
            traj_pi = (np.expand_dims(np.expand_dims(pi, axis=-1), axis=-1) * mu).sum(axis=1)  # [B, T, 2]
            batch_size, K, T, _ = mu.shape
            pred_list = []
            for batch_id in range(batch_size):
                y_pred = {}
                pred_top = mu[batch_id]  # 已经是 numpy 数组
                pi_top = pi[batch_id]  # 已经是 numpy 数组
                for i in range(len(pi_top)):
                    str_score = f'{pi_top[i]:.6f}'
                    while str_score in y_pred:
                        str_score = str_score + '0'  # Append one more '0'
                    y_pred[str_score] = pred_top[i]

                y_pred['1'] = traj_pi[batch_id]
                origin = origs[batch_id]
                angle = -angles[batch_id]
                for key, val in y_pred.items():
                    if is_rotate:
                        y_pred[key] = self.rotate_xy_numpy(y_pred[key], angle)
                    y_pred[key] += origin
                pred_list.append([seq_ids[batch_id],self_array[batch_id], y_pred])

            return pred_list
        return None

    def infer(self, data):
        if self.session_tnt is not None:
            # Run ONNX inference
            inputs = {
                # 'batch': to_numpy(data.batch),  # shape (num_nodes, feat_dim)
                'x': to_numpy(data.x),  # shape (num_nodes, feat_dim)
                'cluster': to_numpy(data.cluster).astype(np.int64),
                'edge_index': to_numpy(data.edge_index).astype(np.int64),
                'identifier': to_numpy(data.identifier).astype(np.float32),
                'candidate': to_numpy(data.candidate).astype(np.float32),
                'candidate_mask': to_numpy(data.candidate_mask).astype(np.float32),
                'candidate_len_max': to_numpy(data.candidate_len_max).astype(np.int64),
                'time_step_len': to_numpy(data.time_step_len).astype(np.int64),
                'valid_len': to_numpy(data.valid_len).astype(np.int64),
            }
            # ['x', 'cluster', 'identifier', 'candidate', 'candidate_len_max', 'time_step_len', 'valid_len']
            batch_size = data.valid_len.size(0)

            # inputs['candidate'] = inputs['candidate'].reshape(batch_size, -1, 2)
            # inputs['candidate_mask'] = inputs['candidate_mask'].reshape(batch_size, -1, 1)

            feed = {name: inputs[name] for name in self.input_name_tnt if name in inputs}
            outs = self.session_tnt.run(None, feed)
            # traj_pred = outs
            # traj_pred = torch.from_numpy(traj_pred.astype(np.float32)).float()

            traj_pred, score_pred = outs
            traj_pred = torch.from_numpy(traj_pred.astype(np.float32)).float()
            score_pred = torch.from_numpy(score_pred.astype(np.float32)).float()
            traj_selected, score_selected = self.traj_selection(traj_pred, score_pred)
            pred_y = traj_selected.view((batch_size, self.k, self.horizon, 2)).cpu().numpy()
            score_y = score_selected.unsqueeze(len(score_selected.shape)).view((batch_size, self.k)).cpu().numpy()
            origs = data.orig.numpy()
            rots = data.rot.numpy()
            seq_ids = data.seq_id.numpy()

            # end0 = True
            end0 = False
            forecasted_trajectories = {}
            for batch_id in range(batch_size):
                seq_id = seq_ids[batch_id]
                t_ori = origs[batch_id].copy()
                if end0:
                    origs[batch_id] = np.array([0, 0], dtype=np.float32)
                else:
                    origs[batch_id] = t_ori
                # convert to dict which key is score, value is trajectory, and convert the coordinate to global coordinate

                score_list = score_y[batch_id]
                score_list = score_list / np.sum(score_list)
                forecast_list = [self.convert_coord(pred_y_k, origs[batch_id], rots[batch_id])
                                 for pred_y_k in pred_y[batch_id]]
                score_traj = {}
                for i in range(len(score_list)):
                    str_score = f'{score_list[i]:.6f}'
                    while str_score in score_traj:
                        str_score = str_score + '0'  # Append one more '0'
                    score_traj[str_score] = forecast_list[i]
                forecasted_trajectories[seq_id] = score_traj

            return forecasted_trajectories, pred_y, score_y # 1 6 30 2, 1 6
        return None, None

    def convert_coord(self, traj, orig, rot):
        # return traj
        traj_converted = np.matmul(np.linalg.inv(rot), traj.T).T + orig.reshape(-1, 2)
        # traj_converted = np.matmul(np.linalg.inv(rot), traj.T).T
        return traj_converted


    def convert_coord_dict(self, traj_dict, orig, rot):
        for _, nodes in traj_dict.items():
            # _xy = nodes[:, :2]
            converted_xy = self.convert_coord(nodes[:, :2], orig, rot)
            nodes[:, :2] = converted_xy
        return traj_dict

    def rotation_matrix(self, phi: float) -> np.ndarray:
        c, s = np.cos(phi), np.sin(phi)
        return np.array([[c, -s], [s, c]], dtype=float)

    # NumPy：镜像（关于 y 轴） -> x -> -x, y 不变
    def mirror_xy_numpy(self, traj: np.ndarray) -> np.ndarray:
        """
        traj: ndarray, shape (T, 2) 或 (N, T, 2), 最后一维为 [x, y]
        返回同形状数组
        """
        arr = np.asarray(traj).copy()
        arr[..., 0] = -arr[..., 0]
        return arr

    # NumPy：旋转（绕原点或中心）
    def rotate_xy_numpy(self, traj: np.ndarray, phi: float, center=None) -> np.ndarray:
        """
        对 (T,2) 或 (N,T,2) 的数组进行逆时针旋转。
        - phi: 旋转角（弧度）
        - center: (cx, cy) 旋转中心；None 表示原点
        """
        arr = np.asarray(traj).copy()
        R = self.rotation_matrix(phi)
        if center is None:
            xy = arr[..., :2]
            # arr[..., :2] = xy @ R.T
            arr[..., :2] = (R @ xy.T).T
        else:
            cx, cy = center
            xy = arr[..., :2]
            xy_shift = xy - np.array([cx, cy])
            xy_rot = xy_shift @ R.T + np.array([cx, cy])
            arr[..., :2] = xy_rot
        return arr

    def cal_dxy(self, points):
        dt = 0.1  # Time step (seconds)
        dx = np.diff(points[:, 0])  # Δx between points
        dy = np.diff(points[:, 1])  # Δy between points
        dx = np.concatenate((dx, [dx[-1]]))
        dy = np.concatenate((dy, [dy[-1]]))
        return np.column_stack((points, dx, dy))

    def calculate_enhanced_candidate_features(self, hist_traj, cand_traj):
        """
        增强版的候选轨迹特征提取
        返回的特征包括:
        1. 速度特征 (平均速度, 最大速度, 速度方差)
        2. 位置特征 (终点位置, 与历史末帧的距离, 偏移)
        3. 方向特征 (夹角, 方向一致性)
        4. 运动特征 (加速度, 曲率)
        5. 轨迹特征 (长度, 直线度)
        """
        num_candidates = cand_traj.shape[0]
        timesteps = cand_traj.shape[1]
        features_list = []
        # 历史轨迹的末帧信息
        hist_end = hist_traj[-1]
        hist_end_pos = hist_end[:2]  # (x, y)
        hist_end_vel = hist_end[2:]  # (dx, dy)
        hist_speed = np.linalg.norm(hist_end_vel)
        for i in range(num_candidates):
            cand = cand_traj[i]  # shape (30, 4)
            features = []
            # === 1. 速度特征 ===
            cand_vel = cand[:, 2:4]  # (30, 2)
            speeds = np.linalg.norm(cand_vel, axis=1)
            # 平均速度
            avg_speed = speeds.mean()
            features.append(avg_speed)
            # 最大速度
            max_speed = speeds.max()
            features.append(max_speed)
            # 最小速度
            min_speed = speeds.min()
            features.append(min_speed)
            # 速度方差
            speed_var = speeds.var()
            features.append(speed_var)
            # 末速度
            end_speed = speeds[-1]
            features.append(end_speed)
            # 速度变化率 (从开始到结束)
            if timesteps > 1:
                speed_change = speeds[-1] - speeds[0]
            else:
                speed_change = 0.0
            features.append(speed_change)

            # === 2. 位置特征 ===
            cand_pos = cand[:, :2]  # (30, 2)
            cand_start = cand_pos[0]
            cand_end = cand_pos[-1]
            # 终点位置
            features.append(cand_end[0])  # end_x
            features.append(cand_end[1])  # end_y
            # 与历史末帧的距离
            end_to_hist_dist = np.linalg.norm(cand_end - hist_end_pos)
            features.append(end_to_hist_dist)
            # 起点与历史末帧的距离
            start_to_hist_dist = np.linalg.norm(cand_start - hist_end_pos)
            features.append(start_to_hist_dist)
            # 相对于历史末帧的偏移
            offset = cand_end - hist_end_pos
            features.append(offset[0])  # offset_x
            features.append(offset[1])  # offset_y
            # === 3. 方向特征 ===
            # 历史末帧速度方向
            if hist_speed > 1e-6:
                hist_direction = hist_end_vel / hist_speed
            else:
                hist_direction = np.array([0.0, 0.0])
            # 候选轨迹末帧方向
            cand_end_vel = cand_vel[-1]
            cand_end_speed = np.linalg.norm(cand_end_vel)
            if cand_end_speed > 1e-6:
                cand_end_direction = cand_end_vel / cand_end_speed
            else:
                cand_end_direction = np.array([0.0, 0.0])

            # 方向夹角
            if hist_speed > 1e-6 and cand_end_speed > 1e-6:
                cos_angle = np.dot(hist_direction, cand_end_direction)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                direction_angle = np.degrees(np.arccos(cos_angle))
            else:
                direction_angle = 0.0
            features.append(direction_angle)

            # 与历史末帧到终点的向量的夹角
            hist_to_end = cand_end - hist_end_pos
            hist_to_end_norm = np.linalg.norm(hist_to_end)
            if hist_to_end_norm > 1e-6 and cand_end_speed > 1e-6:
                hist_to_end_dir = hist_to_end / hist_to_end_norm
                cos_angle2 = np.dot(cand_end_direction, hist_to_end_dir)
                cos_angle2 = np.clip(cos_angle2, -1.0, 1.0)
                angle2 = np.degrees(np.arccos(cos_angle2))
            else:
                angle2 = 0.0
            features.append(angle2)
            # === 4. 运动特征 ===
            # 平均加速度
            if timesteps > 1:
                acc = np.diff(cand_vel, axis=0)
                acc_magnitudes = np.linalg.norm(acc, axis=1)
                avg_acc = acc_magnitudes.mean() if len(acc_magnitudes) > 0 else 0.0
            else:
                avg_acc = 0.0
            features.append(avg_acc)
            # 最大加速度
            max_acc = acc_magnitudes.max() if timesteps > 1 else 0.0
            features.append(max_acc)
            # 曲率
            if timesteps > 2:
                directions = cand_vel / (np.linalg.norm(cand_vel, axis=1, keepdims=True) + 1e-6)
                direction_changes = np.arccos(np.clip(
                    np.sum(directions[1:] * directions[:-1], axis=1), -1.0, 1.0
                ))
                avg_curvature = np.degrees(direction_changes.mean())
            else:
                avg_curvature = 0.0
            features.append(avg_curvature)
            # === 5. 轨迹特征 ===
            # 轨迹长度
            if timesteps > 1:
                segments = np.diff(cand_pos, axis=0)
                seg_lengths = np.linalg.norm(segments, axis=1)
                total_length = seg_lengths.sum()
            else:
                total_length = 0.0
            features.append(total_length)
            # 直线度 (起点到终点的距离 / 轨迹总长度)
            if total_length > 1e-6:
                direct_distance = np.linalg.norm(cand_end - cand_start)
                straightness = direct_distance / total_length
            else:
                straightness = 0.0
            features.append(straightness)

            # 轨迹的包围盒大小
            min_coords = cand_pos.min(axis=0)
            max_coords = cand_pos.max(axis=0)
            bbox_size = np.linalg.norm(max_coords - min_coords)
            features.append(bbox_size)
            features_list.append(features)
        arr = np.array(features_list, dtype=np.float32)
        has_nan = np.any(np.isnan(arr))
        if has_nan:
            print(arr)
        return np.array(features_list, dtype=np.float32)

    def distance_metric(self, traj_candidate: torch.Tensor, traj_gt: torch.Tensor):
        """
        compute the distance between the candidate trajectories and gt trajectory
        :param traj_candidate: torch.Tensor, [batch_size, M, horizon * 2] or [M, horizon * 2]
        :param traj_gt: torch.Tensor, [batch_size, horizon * 2] or [1, horizon * 2]
        :return: distance, torch.Tensor, [batch_size, M] or [1, M]
        """
        assert traj_gt.dim() == 2, "Error dimension in ground truth trajectory"
        if traj_candidate.dim() == 3:
            pass

        elif traj_candidate.dim() == 2:
            traj_candidate = traj_candidate.unsqueeze(1) # 单样本输入添加中间维度（例如 [6, 30] -> [6, 1, 30]）
        else:
            raise NotImplementedError
        assert traj_candidate.size()[2] == traj_gt.size()[1], "Miss match in prediction horizon!"
        batch_size, M, horizon_2_times = traj_candidate.size()
        # 相同位置坐标相减并平方
        dis = torch.pow(traj_candidate - traj_gt.unsqueeze(1), 2).view(-1, M, int(horizon_2_times / 2), 2)
        # 计算每个时间步的欧氏距离 对xy坐标求和 (dx² + dy²),  取所有时间步的最大值
        # dis, _ = torch.max(torch.sum(torch.sqrt(dis), dim=3).cumsum(axis=2), dim=2) #torch.sqrt
        dis, _ = torch.max(torch.sum(dis, dim=3), dim=2)
        return dis

    def traj_selection(self, traj_in, score, threshold=16):
        """
        select the top k trajectories according to the score and the distance
        :param traj_in: candidate trajectories, [batch, M, horizon * 2]
        :param score: score of the candidate trajectories, [batch, M]
        :param threshold: float, the threshold for exclude traj prediction
        :return: [batch_size, k, horizon * 2]
        """
        # re-arrange trajectories according the the descending order of the score
        _, batch_order = score.sort(descending=True)
        traj_pred = torch.cat([traj_in[i, order] for i, order in enumerate(batch_order)], dim=0).view(-1, self.m,
                                                                                                      self.horizon * 2)
        traj_selected = traj_pred[:, :self.k].clone()  # [batch_size, k, horizon * 2]
        score_pred = torch.cat([score[i, order] for i, order in enumerate(batch_order)], dim=0).view(-1, self.m)
        score_selected = score_pred[:, :self.k].clone()
        # check the distance between them, NMS, stop only when enough trajs collected
        for batch_id in range(traj_pred.shape[0]):  # one batch for a time
            traj_cnt = 1
            thres = threshold
            while traj_cnt < self.k:
                for j in range(1, self.m):
                    dis = self.distance_metric(traj_selected[batch_id, :traj_cnt], traj_pred[batch_id, j].unsqueeze(0))
                    if not torch.any(dis < thres):
                        traj_selected[batch_id, traj_cnt] = traj_pred[batch_id, j].clone()
                        score_selected[batch_id, traj_cnt] = score[batch_id][batch_order[batch_id][j]]
                        traj_cnt += 1
                    if traj_cnt >= self.k:
                        break
                thres /= 2.0
        return traj_selected, score_selected

if __name__ == "__main__":
    pass

