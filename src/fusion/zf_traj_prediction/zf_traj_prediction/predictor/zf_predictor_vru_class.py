from .zf_predictor_base import BasePredictor
from zf_traj_prediction.zf_common import EAgentProp
import numpy as np
from collections import deque
import os
import time
from ament_index_python.packages import get_package_share_directory
import os
import onnxruntime as ort
from typing import Tuple, Optional
from .tnt_online_dataset import DataInMem
from .tnt_online_infer import TNTOnlineInfer as TNTOnlineInferStandard, EAgentType, GraphData
from .tnt_online_infer_opt_NMS import TNTOnlineInfer as TNTOnlineInferOptNMS


class VRUClassPredictor(BasePredictor):
    def __init__(self, model_path=None, tnt_infer_impl: str = 'standard'):
        """model_path: optional path to trained model (torch .pth / .pt or pickle).
        device: 'cpu' or 'cuda' (cpu used by default).
        """
        super().__init__()
        self.dt = 0.1
        self.history = deque(maxlen=30)
        self.model = None
        # 分类模型配置
        pkg_share = get_package_share_directory('zf_traj_prediction')
        model_dir = os.path.join(pkg_share, 'model')
        self.class_onnx_path = os.path.join(model_dir, 'VRUClassifier.onnx')
        # x, y, speed, acceleration, yaw_rate, curvature, cos_heading, sin_heading
        # self.num_classes = 3  # pedestrian, bicycle, motorcycle (根据你的实际类别数调整)
        self.class_names = ["pedestrian", "bicycle", "motorcycle"]
        # 加载分类ONNX模型
        self.class_session = None
        if os.path.exists(self.class_onnx_path):
            try:
                self.class_session = ort.InferenceSession(
                    self.class_onnx_path,
                    providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
                )
            except Exception as e:
                print(f"[TNTPredictor] Failed to load ONNX: {e}")

    def predict_traj(self, agent, surroundings):

        return self.predict_class(agent)

    def predict_class(self, agent, use_model=True) -> Tuple[int, float]:
        """使用ONNX模型预测VRU类别
        Returns:
            Tuple[str, float]: (预测的类别名称, 置信度)
        """
        if not use_model:
            return self.is_ped(agent), 1.0
        if self.class_session is None:
            print("[TNTPredictor] Warning: Classification ONNX not loaded, returning default")
            return EAgentType.PEDESTRIAN.value, 0.0
        history_seq, summary_features = self.extract_classification_features(agent)
        inputs = {
            'history_seq': history_seq[np.newaxis, ...].astype(np.float32),
            'summary_features': summary_features[np.newaxis, ...].astype(np.float32)
        }
        try:
            outputs = self.class_session.run(None, inputs)
            logits = outputs[0]  # 假设输出是logits
            exp_logits = np.exp(logits - np.max(logits, axis=-1, keepdims=True))
            probs = exp_logits / np.sum(exp_logits, axis=-1, keepdims=True)
            class_idx = np.argmax(logits, axis=-1)[0]
            confidence = probs[0, class_idx]
            # predicted_class = self.class_names[class_idx] if class_idx < len(self.class_names) else "unknown"
            # if class_idx == 1:
            #     return self.is_ped(agent), 1.0
            return class_idx, float(confidence) # EAgentType(class_idx)
        except Exception as e:
            print(f"[TNTPredictor] ONNX inference failed: {e}")
            return EAgentType.PEDESTRIAN.value, 0.0

    def is_ped(self, points):
        """快速分类，适用于实时系统"""
        n = len(points)
        if n < 3:
            return EAgentType.BICYCLE.value
        # 提取速度和位置
        v = points[:, EAgentProp.V].astype(np.float64)
        x = points[:, EAgentProp.X].astype(np.float64)
        y = points[:, EAgentProp.Y].astype(np.float64)

        # 计算关键指标
        mean_speed = np.mean(v)
        max_speed = np.max(v)

        # 计算路径直度
        total_dist = np.sum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2))
        straight_dist = np.sqrt((x[-1] - x[0]) ** 2 + (y[-1] - y[0]) ** 2)
        straightness = straight_dist / total_dist if total_dist > 0 else 0
        # 简单规则
        if mean_speed < 1.5 and max_speed < 3:
            return EAgentType.PEDESTRIAN.value
        elif mean_speed < 2.0:
            if mean_speed > 1.5 and straightness > 0.75:
                return EAgentType.BICYCLE.value
            return EAgentType.PEDESTRIAN.value
        return EAgentType.BICYCLE.value

    def extract_classification_features(self, agent) -> Tuple[np.ndarray, np.ndarray]:
        history_xy = agent[:, [EAgentProp.X, EAgentProp.Y]].astype(np.float32)
        if history_xy.shape[0] < 2:
            history_xy = np.repeat(history_xy[-1:], 2, axis=0)
        history_xy = self._fix_length(history_xy, target_length=30)
        normalized_xy = self._normalize_xy(history_xy)
        history_seq, summary = self._build_history_features(normalized_xy)
        return history_seq, summary

    def _fix_length(self, history_xy: np.ndarray, target_length: int = 30) -> np.ndarray:
        """固定历史长度（参考class_data_loader.py）"""
        if history_xy.shape[0] == target_length:
            return history_xy.copy()
        if history_xy.shape[0] > target_length:
            return history_xy[-target_length:].copy()
        pad_count = target_length - history_xy.shape[0]
        padding = np.repeat(history_xy[:1], pad_count, axis=0)
        return np.concatenate([padding, history_xy], axis=0)

    def _normalize_xy(self, history_xy: np.ndarray) -> np.ndarray:
        translated = history_xy - history_xy[-1]
        # 计算朝向向量（最后一帧到第一帧）
        heading_vector = translated[-1] - translated[0]
        if np.linalg.norm(heading_vector) < 1e-6:
            # 如果首尾点重合，使用最大速度方向
            diffs = np.diff(translated, axis=0)
            magnitudes = np.linalg.norm(diffs, axis=1)
            best_idx = int(np.argmax(magnitudes)) if len(magnitudes) else 0
            heading_vector = diffs[best_idx] if len(diffs) else np.array([1.0, 0.0], dtype=np.float32)
        # 计算旋转角度
        heading_angle = np.arctan2(heading_vector[1], heading_vector[0])
        cos_val = np.cos(-heading_angle)
        sin_val = np.sin(-heading_angle)
        rotation = np.array([[cos_val, -sin_val], [sin_val, cos_val]], dtype=np.float32)
        return translated @ rotation.T

    def _build_history_features(self, history_xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        diffs = np.diff(history_xy, axis=0, prepend=history_xy[:1])
        velocity = diffs / self.dt
        velocity[0] = velocity[1] if len(velocity) > 1 else velocity[0]
        speed = np.linalg.norm(velocity, axis=1)
        heading = np.arctan2(velocity[:, 1], velocity[:, 0])
        heading = np.unwrap(heading)
        yaw_rate = np.diff(heading, prepend=heading[:1]) / self.dt
        acceleration = np.diff(speed, prepend=speed[:1]) / self.dt
        segment_lengths = np.linalg.norm(np.diff(history_xy, axis=0), axis=1)
        path_length = float(segment_lengths.sum())
        displacement = float(np.linalg.norm(history_xy[-1] - history_xy[0]))
        straightness = displacement / (path_length + 1e-6)
        bbox_extent = history_xy.max(axis=0) - history_xy.min(axis=0)
        curvature = np.abs(yaw_rate) / (speed + 1e-3)
        # (30, 8)
        history_seq = np.column_stack([
            history_xy[:, 0],  # x
            history_xy[:, 1],  # y
            speed,
            acceleration,
            yaw_rate,
            curvature,
            np.cos(heading),
            np.sin(heading),
        ]).astype(np.float32)

        summary = np.array([
            speed.mean(),
            speed.std(),
            speed.max(),
            acceleration.mean(),
            np.abs(acceleration).max(),
            np.abs(yaw_rate).mean(),
            yaw_rate.std(),
            curvature.mean(),
            path_length,
            displacement,
            straightness,
            float(np.linalg.norm(bbox_extent)),
        ], dtype=np.float32)
        return history_seq, summary

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
        track_ids = feat[:, EAgentProp.TRACK_ID]
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
                          EAgentProp.TRAFFIC, EAgentProp.LANE_LEFT, EAgentProp.LANE_RIGHT, EAgentProp.INTERSECTION,
                          EAgentProp.TRACK_ID,
                          EAgentProp.CLASS]].astype(np.float32)
        result = np.nan_to_num(result, nan=0.0)
        return result


if __name__ == "__main__":
    a = VRUClassPredictor()