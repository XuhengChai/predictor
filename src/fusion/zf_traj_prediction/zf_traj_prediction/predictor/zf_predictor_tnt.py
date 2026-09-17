from .zf_predictor_base import BasePredictor
from zf_traj_prediction.zf_common import EAgentProp
import numpy as np
from collections import deque
import pickle
import time
from ament_index_python.packages import get_package_share_directory
import os
from .tnt_online_dataset import DataInMem
from .tnt_online_infer import TNTOnlineInfer as TNTOnlineInferStandard, EAgentType, GraphData
from .tnt_online_infer_opt_NMS import TNTOnlineInfer as TNTOnlineInferOptNMS


def build_tnt_infer(impl: str, onnx_tnt: str, onnx_mdn: str, device: str):
    impl_key = impl.lower()
    infer_cls_map = {
        "standard": TNTOnlineInferStandard,
        "default": TNTOnlineInferStandard,
        "base": TNTOnlineInferStandard,
        "opt_nms": TNTOnlineInferOptNMS,
        "opt-nms": TNTOnlineInferOptNMS,
    }
    if impl_key not in infer_cls_map:
        raise ValueError(f"Unsupported TNT infer implementation: {impl}")
    return infer_cls_map[impl_key](onnx_tnt=onnx_tnt, onnx_mdn=onnx_mdn, device=device)

class TNTPredictor(BasePredictor):
    def __init__(self, model_path=None, device='cpu', tnt_infer_impl: str = 'standard'):
        """model_path: optional path to trained model (torch .pth / .pt or pickle).
        device: 'cpu' or 'cuda' (cpu used by default).
        """
        super().__init__()
        self.dt = 0.1
        self.history = deque(maxlen=30)
        self.model = None
        self.device = device
        self.tnt_infer_impl = tnt_infer_impl
        pkg_share = get_package_share_directory('zf_traj_prediction')
        model_dir = os.path.join(pkg_share, 'model')
        tnt_onnx = os.path.join(model_dir, 'TNT.onnx')
        mdn_onnx = os.path.join(model_dir, 'MDNModel.onnx')
        range_onnx = os.path.join(model_dir, 'RangePred.onnx')
        self.data_process = DataInMem(range_onnx=range_onnx)
        self.data_infer = build_tnt_infer(
            impl=tnt_infer_impl,
            onnx_tnt=tnt_onnx,
            onnx_mdn=mdn_onnx,
            device=device,
        )

    import numpy as np

    def inverse_calc_traj_bezier_np(self, end_x, end_y, psi0, T=3.0, num_frames=30):
        """
        end_x, end_y: [K]
        psi0: 标量
        return: [K, num_frames, 2]
        """
        K = end_x.shape[0]
        P0 = np.zeros((K, 2))
        P2 = np.stack([end_x, end_y], axis=-1)  # [K,2]
        heading = np.stack([np.cos(psi0), np.sin(psi0)])  # [2]
        heading = np.tile(heading, (K, 1))  # [K,2]
        dist = np.linalg.norm(P2 - P0, axis=-1, keepdims=True)  # [K,1]
        # 5. 控制点
        alpha = 0.6
        P1 = P0 + heading * dist * alpha  # [K,2]
        t = np.linspace(0.0, 1.0, num_frames).reshape(1, num_frames, 1)
        traj = (
                (1 - t) ** 2 * P0[:, None, :] +
                2 * (1 - t) * t * P1[:, None, :] +
                t ** 2 * P2[:, None, :]
        )  # [K, num_frames, 2]
        return traj

    def predict_traj(self, agent, surroundings):
        """Main entry: returns a dict mapping probability -> traj (np.array shape [T,2]).
        If a loaded model is present and callable it will be used. Otherwise fallback
        to a simple constant-velocity extrapolation producing a single entry with prob 1.0.
        """
        # start = time.perf_counter()
        input_feat = self.process_traj(agent, surroundings)
        # Run model inference
        feat_list = [input_feat]
        data = self.data_process.process_feats_list(feat_list)
        pred_traj = self.data_infer.infer_with_mdn(data)
        # end = time.perf_counter()
        # print(f"执行时间: {end - start:.6f} 秒")
        # Format output: dict of prob -> traj
        y_pred = None
        for i in range(len(feat_list)):
            seq_id, history, y_pred = pred_traj[i]
            # # TODO multi-agent: currently we only handle the first agent in the input, but we should ideally match seq_id to track_id
            # track_id = agent[0, EAgentProp.TRACK_ID]
            # frame_id = agent[-1, EAgentProp.FRAME_ID]
        for k, traj in y_pred.items():
            new_traj_global = self.rebuild_predicted_path_from_endpoint(
                agent, traj[-1], num_frames=30
            )
            # # 3. 原轨迹终点 -> 转为相对坐标  bezier
            # last_xy = agent[-1, :2]  # (2,)
            # vx, vy = agent[-1, 2:4]
            # psi0 = np.arctan2(vy, vx)
            # end_point = traj[-1]  # (2,)
            # # rel = end_point - last_xy  # (2,)
            # # end_x = np.array([rel[0]])
            # # end_y = np.array([rel[1]])
            # # new_traj = self.inverse_calc_traj_bezier_np(
            # #     end_x, end_y, psi0, num_frames=30
            # # )[0]  # [30,2]
            # # new_traj_global = new_traj + last_xy
            y_pred[k] = new_traj_global
        return y_pred

    def _build_input(self, agent, surroundings):
        # Ensure 2D arrays
        agent_arr = np.asarray(agent)
        surr_arr = np.asarray(surroundings)

        # Select relevant features: X, VX, Y, VY (consistent with other predictors)
        def pick_xyvx(arr):
            if arr.size == 0:
                return np.zeros((0, 4), dtype=float)
            # If arr has fewer columns than expected, pad with zeros
            ncols = arr.shape[1]
            needed = max(EAgentProp.VY.value, EAgentProp.V.value) + 1 if hasattr(EAgentProp, 'V') else EAgentProp.VY + 1
            # But simpler: use indices directly
            idxs = [EAgentProp.X, EAgentProp.VX, EAgentProp.Y, EAgentProp.VY]
            cols = []
            for i in idxs:
                if i.value < ncols:
                    cols.append(arr[:, i.value].astype(float))
                else:
                    cols.append(np.zeros(arr.shape[0], dtype=float))
            stacked = np.stack(cols, axis=1)
            return stacked

        a_feat = pick_xyvx(agent_arr)
        s_feat = pick_xyvx(surr_arr)

        # Flatten: agent first (time-major), then surroundings (agent-ordered)
        a_flat = a_feat.flatten()
        s_flat = s_feat.flatten()
        inp = np.concatenate([a_flat, s_flat]).astype(np.float32)
        return inp.reshape(1, -1)

    def process_traj(self, agent, surroundings):
        # keep consistent with other predictors (X, VX, Y, VY)
        feat = np.concatenate([agent, surroundings])
        track_ids = feat[:,  EAgentProp.TRACK_ID]
        # 使用np.unique获取唯一的track_id和它们在原始数组中对应的新编号 ordered
        # unique_ids, inverse_indices = np.unique(original_track_ids, return_inverse=True)
        changes = np.where(track_ids[:-1] != track_ids[1:])[0] + 1
        unique_ids_ordered = track_ids[np.concatenate(([0], changes))]
        # 批量创建映射
        from_to = np.vstack([unique_ids_ordered, np.arange(len(unique_ids_ordered))]).T
        track_dict = dict(from_to)
        new_ids = np.vectorize(track_dict.get)(track_ids)
        feat[:, EAgentProp.TRACK_ID] = new_ids
        feat[:, EAgentProp.VX] *= self.dt
        feat[:, EAgentProp.VY] *= self.dt
        feat[:, EAgentProp.FRAME_ID] = feat[:, EAgentProp.FRAME_ID] - feat[0, EAgentProp.FRAME_ID]
        result = feat[:, [EAgentProp.X, EAgentProp.Y, EAgentProp.VX, EAgentProp.VY, EAgentProp.FRAME_ID,
                          EAgentProp.TRAFFIC, EAgentProp.LANE_LEFT, EAgentProp.LANE_RIGHT, EAgentProp.INTERSECTION, EAgentProp.TRACK_ID,
                          EAgentProp.CLASS]].astype(np.float32)
        result = np.nan_to_num(result, nan=0.0)
        return result


if __name__ == "__main__":
    a = TNTPredictor()