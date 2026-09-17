import os
import re
import math
import numpy as np
from tqdm import tqdm

import gc
from copy import deepcopy, copy
import pickle
import torch
from torch_geometric.data import Data, Dataset, InMemoryDataset, DataLoader
# from torch.utils.data import DataLoader
from enum import Enum, unique, IntEnum
import onnxruntime as ort
from .tnt_online_infer import TNTOnlineInfer, EAgentType, GraphData


def print_np(M):
    M_str = np.array2string(M, separator=',', formatter={'float': lambda x: "%.8f" %x})
    print("[", M_str[1:-1], "]")

def get_fc_edge_index(node_indices):
    """
    node_indices: np.array([indices]), the indices of nodes connecting with each other;
    return a tensor(2, edges), indicing edge_index
    """
    xx, yy = np.meshgrid(node_indices, node_indices)
    xy = np.vstack(([xx.reshape(-1), yy.reshape(-1)])).astype(np.int64)
    return xy

def get_traj_edge_index(node_indices):
    """
    generate the polyline graph for traj, each node are only directionally connected with the nodes in its future
    node_indices: np.array([indices]), the indices of nodes connecting with each other;
    return a tensor(2, edges), indicing edge_index
    """
    edge_index = np.empty((2, 0))
    for i in range(len(node_indices)):
        xx, yy = np.meshgrid(node_indices[i], node_indices[i:])
        edge_index = np.hstack([edge_index, np.vstack(([xx.reshape(-1), yy.reshape(-1)])).astype(np.int64)])
    return edge_index



# %%
@unique
class ECombineType(Enum):
    ORIGIN = 0
    SELF_SURROUND = 1
    SELF_ROAD = 2
    SELF = 3
    ORIGIN_DEL = 4



# dataset loader which loads data into memory
class DataInMem(InMemoryDataset):
    def __init__(self, root = "", transform=None, pre_transform=None, range_onnx=None, save_processed=False):
        self.history_len = 30
        load_path = range_onnx if (range_onnx is not None and len(range_onnx) > 0) else r"RangePred.onnx"
        self.model = ort.InferenceSession(load_path,
                                          providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
        # print("Loading data from {}...".format(self.processed_paths[0]))
        if root and save_processed and os.path.exists(self.processed_paths[0]):
            super(DataInMem, self).__init__(root, transform, pre_transform)
            self.data, self.slices = torch.load(self.processed_paths[0])
        gc.collect()

    @property
    def raw_file_names(self):
        return [file for file in os.listdir(self.raw_dir) if file.endswith(".pkl")]

    @property
    def processed_file_names(self):
        return ['data.pt']

    def download(self):
        pass

    # def process(self):
    #     pass

    def split_featsx(self, featsx, traj_lens):
        ids = featsx[:, 9]
        self_array = featsx[ids == 0]
        surrounding_array = featsx[(ids >= 1) & (ids <= traj_lens - 1)]
        road_array = featsx[ids >= traj_lens]
        return self_array, surrounding_array, road_array

    def process_feats(self, raw_data, type=ECombineType.ORIGIN):
        if type == ECombineType.ORIGIN: # nothing
            return
        feats_x, surrounding_feats, \
            road_feats = self.split_featsx(raw_data["feats_x"], raw_data['traj_lens'])
        traj_cnt = 1
        if type == ECombineType.SELF_SURROUND:
            traj_cnt = raw_data['traj_lens']
            feats_x = np.vstack([feats_x, surrounding_feats])
        elif type == ECombineType.SELF_ROAD:
            road_feats[:, -1] -= (raw_data['traj_lens'] - 1)
            feats_x = np.vstack([feats_x, road_feats])
        raw_data["feats_x"] = feats_x
        raw_data["traj_lens"] = traj_cnt

    def generate_candidate_trajectories(self, history_trajectory, pre_y, n_trajectories=20, n_points=30,
                                            dt=0.1):
        x0, y0 = history_trajectory[-1][:2]
        v1 = [y0 - history_trajectory[0][1], x0 - history_trajectory[0][0]]
        v2 = [y0 - history_trajectory[-2][1], x0 - history_trajectory[-2][0]]
        yaw_list = np.arctan2(v2[0], v2[1])

        fact = 0
        v_min = pre_y[0] - fact * abs(pre_y[0])
        v_max = pre_y[1] + fact * abs(pre_y[1])
        yaw_rate_min = pre_y[2] - fact * abs(pre_y[2])
        yaw_rate_max = pre_y[3] + fact * abs(pre_y[3])

        v_perturb = np.linspace(v_min, v_max, n_trajectories)
        yaw_rate_perturb = np.linspace(yaw_rate_min, yaw_rate_max, n_trajectories)
        # Initialize trajectory tensor
        start_p = 0
        trajectories_final = np.zeros((n_trajectories, n_trajectories, 5))
        for i in range(n_trajectories):
            # Apply DWA sampling for this trajectory
            traj_v = v_perturb[i]
            # traj_v = max(v, float(v_perturb[i]))
            # traj_v = v_max
            for k in range(n_trajectories):
                traj_yaw_rate = yaw_rate_perturb[k]
                yaw = yaw_list
                cur_x, cur_y = x0, y0
                current_yaw = yaw
                for j in range(1, n_points + start_p):
                    current_yaw = current_yaw + traj_yaw_rate * dt
                    cur_x = cur_x + traj_v * np.cos(current_yaw) * dt
                    cur_y = cur_y + traj_v * np.sin(current_yaw) * dt
                trajectories_final[i, k - start_p] = [cur_x, cur_y, traj_v, current_yaw, traj_yaw_rate]
        trajectories_final = trajectories_final.reshape(-1, 5)
        trajectories_final = trajectories_final[:, :2]
        return trajectories_final

    def cal_vxy_yr(self, points):
        dt = 0.1  # Time step (seconds)
        # dx = np.diff(points[:, 0])  # Δx between points
        # dy = np.diff(points[:, 1])  # Δy between points
        dx = points[:, 2]  # Δx between points
        dy = points[:, 3]  # Δy between points
        vx = dx / dt
        vy = dy / dt
        # vx = np.concatenate((vx, [vx[-1]]))
        # vy = np.concatenate((vy, [vy[-1]]))
        v = np.sqrt(vx ** 2 + vy ** 2)
        yaw = np.arctan2(vy, vx)  # Returns angle in [-π, π]
        # points[:, 2] = v
        yaw = np.unwrap(yaw, discont=np.pi)
        # points[:, 3] = yaw
        yaw_rate = np.zeros_like(yaw)
        yaw_diff = np.diff(yaw)
        yaw_diff = np.mod(yaw_diff + np.pi, 2 * np.pi) - np.pi
        yaw_rate[1:] = yaw_diff / dt  # 第一行设为0（因为没有前一时刻）
        points[:, 4] = yaw_rate
        points[:, 2] = vx
        points[:, 3] = vy
        return points[:self.history_len, :5]

    def process_sampling(self, raw_data, cls_id):
        featx, _, _ = self.split_featsx(raw_data["feats_x"], raw_data['traj_lens'])
        feat_xy_vxy_yr = self.cal_vxy_yr(featx)
        # Add simple type flags if raw_path given
        is_pedestrian = (cls_id == EAgentType.PEDESTRIAN)
        is_motorcycle_tricycle = (cls_id == EAgentType.MOTORCYCLIST)
        is_bicycle = (cls_id == EAgentType.BICYCLE)
        features = np.full((len(feat_xy_vxy_yr), 3), [is_pedestrian, is_motorcycle_tricycle, is_bicycle])
        feat_sampling_input = np.concatenate((feat_xy_vxy_yr, features), axis=1)
        feat_sampling_input[:, :2] = feat_sampling_input[:, :2] - feat_sampling_input[-1, :2]
        dxy = feat_sampling_input[0] - feat_sampling_input[-1]
        angle = -np.arctan2(dxy[1], dxy[0])
        feat_sampling_input[:, :2] = self.rotate_xy_numpy(feat_sampling_input[:, :2], angle, feat_sampling_input[-1, :2])
        feat_sampling_input[:, 2:4] = self.rotate_xy_numpy(feat_sampling_input[:, 2:4], angle)
        feat_sampling_input[:, 2:4] *= 10
        # Run ONNX session (self.model is an ort.InferenceSession)
        try:
            inp = feat_sampling_input.astype(np.float32)[None, :, :]
            out = self.model.run(None, {self.model.get_inputs()[0].name: inp})
            pred_y = np.array(out[0])  # batch, 4
        except Exception:
            raise RuntimeError("ONNX inference failed for sample  {}".format(cls_id))
        # TODO consider add batch dimension for sampling
        candidate = self.generate_candidate_trajectories(feat_xy_vxy_yr, pred_y[0])
        raw_data["tar_candts"] = candidate

    def rotation_matrix(self, phi: float) -> np.ndarray:
        c, s = np.cos(phi), np.sin(phi)
        return np.array([[c, -s], [s, c]], dtype=float)

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

    def process_feats_list(self, feats_list=None):
        """
        featx: [xs, ys, vec_x, vec_y, step(timestamp), traffic_control, turn, is_intersection, polyline_id, cls_id];
        """
        p_type = ECombineType.SELF_SURROUND
        traj_lens = []
        valid_lens = []
        candidate_lens = []
        for origin_feat in feats_list:
            ids = origin_feat[:, 9].astype(np.int64)
            unique_ids = np.unique(ids)
            total_len = len(unique_ids)
            traj = origin_feat[:, 4].astype(np.int64) # frame_id
            traj_len = len(traj[traj == self.history_len-1])
            if p_type == ECombineType.ORIGIN or p_type == ECombineType.ORIGIN_DEL:
                traj_lens.append(traj_len)
                valid_lens.append(total_len)
            elif p_type == ECombineType.SELF_SURROUND:
                traj_lens.append(traj_len)
                valid_lens.append(total_len)
            elif p_type == ECombineType.SELF_ROAD:
                traj_lens.append(1)
                valid_lens.append(total_len - traj_len + 1)
            elif p_type == ECombineType.SELF:
                traj_lens.append(1)
                valid_lens.append(1)
            candidate_num = 400
            candidate_lens.append(candidate_num)
        # num_valid_len_max = np.max(valid_lens) if len(valid_lens) > 0 else 0
        num_valid_len_max = 120
        num_candidate_max = np.max(candidate_lens) if len(candidate_lens) > 0 else 0
        data_list = []
        seq_id = 0
        for origin_feat in feats_list:
            # Online single-sample processing
            raw_data = {}
            origin_feat = np.asarray(origin_feat).astype(np.float32)
            raw_data['feats_x'] = origin_feat[:, :10].copy()
            # raw_data['feats_x'] = origin_feat.copy()
            traj = origin_feat[:, 4].astype(np.int64) # frame_id
            raw_data['traj_lens'] = len(traj[traj == self.history_len-1])
            origin_p = origin_feat[self.history_len - 1, :2]
            cls_id = origin_feat[self.history_len - 1, 10]
            raw_data['orig'] = origin_p
            raw_data['rot'] = np.array([[1, 0], [0, 1]], dtype=np.float32)
            raw_data['feats_x'][:, :2] = raw_data['feats_x'][:, :2] - origin_p
            # self.process_feats(raw_data, p_type)
            # update raw_data with cluster, edge_index, identifier for original batch
            identifier = np.empty((0, 2))
            edge_index = np.empty((2, 0), dtype=np.int64)
            feats_x = raw_data['feats_x']
            cluster = copy(feats_x[:, 9].astype(np.int64))  ## add feature not -1
            for cluster_idc in np.unique(cluster):
                [indices] = np.where(cluster == cluster_idc)
                identifier = np.vstack([identifier, np.min(feats_x[indices, :2], axis=0)])
                if len(indices) <= 1:
                    continue  # skip if only 1 node
                edge_index = np.hstack([edge_index, get_fc_edge_index(indices)])
            raw_data["cluster"] = cluster
            raw_data["identifier"] = identifier
            raw_data["edge_index"] = edge_index
            raw_data["valid_lens"] = cluster.max() + 1
            # update raw_data with candidate trajectories generated by sampling
            self.process_sampling(raw_data, cls_id)

            graph_input = GraphData(
                x=torch.from_numpy(raw_data['feats_x'].astype(np.float32)).float(),
                y=torch.zeros((raw_data['feats_x'].shape[0], 4), dtype=torch.float32),
                cluster=torch.from_numpy(raw_data.get('cluster', raw_data['feats_x'][:, 9].astype(np.int64))).short(),
                edge_index=torch.from_numpy(raw_data.get('edge_index', np.empty((2, 0))).astype(np.int64)).long(),
                identifier=torch.from_numpy(raw_data.get('identifier', np.zeros((1, 2))).astype(np.float32)).float(),
                traj_len=torch.tensor([raw_data['traj_lens']]).int(),
                valid_len=torch.tensor([raw_data.get('valid_lens', raw_data['traj_lens'])]).int(),
                time_step_len=torch.tensor([num_valid_len_max]).int(),    # the maximum of no. of polyline
                candidate_len_max=torch.tensor([num_candidate_max]).int(),
                candidate_mask=[],
                candidate=torch.from_numpy(raw_data['tar_candts'].astype(np.float32)).float(),
                orig=torch.from_numpy(raw_data['orig'].astype(np.float32)).float().unsqueeze(0),
                rot=torch.from_numpy(raw_data['rot'].astype(np.float32)).float().unsqueeze(0),
                seq_id=torch.tensor([int(seq_id) if seq_id is not None else -1]).int(),
                class_id=torch.tensor([int(cls_id) if cls_id is not None else -1]).int()
            )
            seq_id += 1
            graph_input = self.pad_data(graph_input)
            data_list.append(graph_input)
        self.data, self.slices = self.collate(data_list)
        if getattr(self, "save_processed", False):
            torch.save((self.data, self.slices), self.processed_paths[0])
        return self.data

    def pad_data(self, data):
        feature_len = data.x.shape[1]
        index_to_pad = data.time_step_len[0].item()
        valid_len = data.valid_len[0].item()
        # pad feature with zero nodes
        data.x = torch.cat([data.x, torch.zeros((index_to_pad - valid_len, feature_len), dtype=data.x.dtype)])
        data.cluster = torch.cat([data.cluster, torch.arange(valid_len, index_to_pad, dtype=data.cluster.dtype)]).long()
        data.identifier = torch.cat([data.identifier, torch.zeros((index_to_pad - valid_len, 2), dtype=data.identifier.dtype)])
        # pad candidate and candidate_gt
        num_cand_max = data.candidate_len_max[0].item()
        data.candidate_mask = torch.cat([torch.ones((len(data.candidate), 1)),
                                         torch.zeros((num_cand_max - len(data.candidate), 1))])
        data.candidate = torch.cat([data.candidate[:, :2], torch.zeros((num_cand_max - len(data.candidate), 2))])
        assert data.cluster.shape[0] == data.x.shape[0], "[ERROR]: Loader error!"
        return data

    def get(self, idx):
        data = super(DataInMem, self).get(idx).clone()
        feature_len = data.x.shape[1]
        index_to_pad = data.time_step_len[0].item()
        valid_len = data.valid_len[0].item()
        # pad feature with zero nodes
        data.x = torch.cat([data.x, torch.zeros((index_to_pad - valid_len, feature_len), dtype=data.x.dtype)])
        data.cluster = torch.cat([data.cluster, torch.arange(valid_len, index_to_pad, dtype=data.cluster.dtype)]).long()
        data.identifier = torch.cat([data.identifier, torch.zeros((index_to_pad - valid_len, 2), dtype=data.identifier.dtype)])
        # pad candidate and candidate_gt
        num_cand_max = data.candidate_len_max[0].item()
        data.candidate_mask = torch.cat([torch.ones((len(data.candidate), 1)),
                                         torch.zeros((num_cand_max - len(data.candidate), 1))])
        data.candidate = torch.cat([data.candidate[:, :2], torch.zeros((num_cand_max - len(data.candidate), 2))])
        assert data.cluster.shape[0] == data.x.shape[0], "[ERROR]: Loader error!"
        return data

if __name__ == "__main__":

    # for folder in os.listdir("./data/interm_data"):
    INTERMEDIATE_DATA_DIR = "./dataset/interm_data"
    # INTERMEDIATE_DATA_DIR = "../TntNet/dataset/interm_data"
    online_infer = TNTOnlineInfer(onnx_tnt=r"TNT.onnx", onnx_mdn=r"MDNModel.onnx")
    # TODO cls ID for predicted agent
    for folder in ["test"]:
    # for folder in ["test"]:
        dataset_input_path = os.path.join(INTERMEDIATE_DATA_DIR, f"{folder}_intermediate")

        # dataset = Argoverse(dataset_input_path)
        dataset = DataInMem(dataset_input_path, range_onnx=r"RangePred.onnx").shuffle()
        batch_iter = DataLoader(dataset, batch_size=1, num_workers=1, shuffle=True, pin_memory=False)
        for k in range(1):
            for i, data in enumerate(tqdm(batch_iter, total=len(batch_iter), bar_format="{l_bar}{r_bar}")):
                # pred_y, score_y = online_infer.infer(data)
                mdn = online_infer.infer_with_mdn(data)