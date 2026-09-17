from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Dict, List, Tuple
from .zf_predictor_base import BasePredictor
from zf_traj_prediction.zf_common import EAgentType, EAgentProp, EPredictorName
from .Physical.imm import Imm
from .Physical.kalman_filter import KalmanFilter
from .Physical.physical_model import PhysicalModel
from .NN.score_prediction import MDNModel
import numpy as np
import torch
from ament_index_python.packages import get_package_share_directory
import os

class EDatasource(Enum):
    """行为意图枚举类"""
    STRAIGHT = 0  # 直行
    LEFT_TURN = 1  # 左转
    RIGHT_TURN = 2  # 右转


@dataclass
class Agent_IMM_output:
    """IMM-MDN 单个智能体的候选预测数据。"""
    agent_id: str
    frame_id: int
    time_stamps: float
    hist_traj: np.ndarray
    pred_traj_by_intent: Dict[float, np.ndarray]
    prob: List[float]
    pred_traj: List[np.ndarray]


class IMMMDNPredictor(BasePredictor):
    """IMM-MDN轨迹预测器
    
    使用交互多模型(IMM)和混合密度网络(MDN)进行轨迹预测
    """
    # 常量定义
    NUM_MODELS = 7  # KF模型个数
    PREDICT_STEPS = 30  # 预测步数
    MODEL_NAMES = ['cv', 'ca', 'ct1', 'ct2', 'ct3', 'ct4', 'ct5']
    
    # 默认行为意图概率
    DEFAULT_PROBS = {
        EDatasource.LEFT_TURN: 0.33,
        EDatasource.RIGHT_TURN: 0.333,
        EDatasource.STRAIGHT: 0.3333,
    }
    
    def __init__(self):
        super().__init__()
        self.method = EPredictorName.IMMMDN
        self.agent_id = None
        self.hist_traj = None
        self.frame_id = None
        self.model_probs = None
        self.X_hist_frames = None
        self.X_cand_frames = None
        self.W_frames = None
        self.Cls_frames = None

        self.Y_trajs = None
        self.seq_ids = None

        #模型配置
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        torch.backends.cudnn.benchmark = True if torch.cuda.is_available() else False

        #随机种子设置
        torch.manual_seed(0)
        np.random.seed(0)

        #1.初始化MDN模型
        self.feat_channel = 23
        self.model = MDNModel(feat_dim=self.feat_channel, device=self.device)

        #2.加载预训练模型权重
        pkg_share = get_package_share_directory('zf_traj_prediction')
        model_dir = os.path.join(pkg_share, 'model')
        model_weights_path = os.path.join(model_dir, 'best_MDNModel.pth')  # 替换为实际路径
        self.model.load_state_dict(torch.load(model_weights_path, map_location=self.device))

        #3.模型转移到设备
        self.model.to(self.device)
        
    def _as_batched(self, x, target_ndim: int, device=None, dtype=torch.float32):
        """
        确保张量至少有 target_ndim 个维度，不够就在最前面补 batch 维。
        - x: np.ndarray | torch.Tensor
        - target_ndim: 期望维度个数，例如 hist_seq 需要 3, cand_seq 需要 4
        """
        if isinstance(x, np.ndarray):
            x = torch.from_numpy(x)
        if not torch.is_tensor(x):
            x = torch.tensor(x)
        while x.dim() < target_ndim:
            x = x.unsqueeze(0)  # 在最前面加 B 维
        if device is None:
            device = self.device
        return x.to(device=device, dtype=dtype)



    def predict_traj(self, agent: np.ndarray, surroundings) -> np.ndarray:
        """预测轨迹
        
        Args:
            agent: 历史数据，shape (history_len, num_features)
            surroundings: 周围环境信息
            
        Returns:
            预测轨迹，shape (predict_steps, 2)，列为[x, y]坐标
        """
        if agent is None or len(agent) == 0:
            return np.empty((0, 2))
        agent_history_data = self.process_traj(agent, surroundings)
        

        # 分别遍历三个意图模型，进行预测
        pred_trajs = {}
        for intent_type in EDatasource:
            physical_model = PhysicalModel()
            pred_trajs[intent_type] = self.IMM_pred(agent_history_data, physical_model, intent_type)
        
        # 将IMM预测结果转换为MDN格式
        result = self._IMM_pred_2_MDN(
            pred_trajs[EDatasource.LEFT_TURN],
            pred_trajs[EDatasource.RIGHT_TURN],
            pred_trajs[EDatasource.STRAIGHT]
        )
        #MDN特征处理及提取
        self._MDN_data_process(result)
        #MDN预测
        pred_traj = self._MDN_predict(agent)


        return pred_traj


    def process_traj(self, agent: np.ndarray, surroundings) -> np.ndarray:
        """处理轨迹数据，提取位置和速度特征
        
        Args:
            agent: 历史数据数组
            surroundings: 周围环境
            
        Returns:
            (N, 4)的特征数组：[x, vx, y, vy]，dtype为float32
        """
        hist_x = agent[:, EAgentProp.X.value]
        hist_y = agent[:, EAgentProp.Y.value]
        hist_vx = agent[:, EAgentProp.VX.value]
        hist_vy = agent[:, EAgentProp.VY.value]
        
        #获取agent的信息
        self.agent_id = agent[0, EAgentProp.TRACK_ID.value]
        self.frame_id = agent[0, EAgentProp.FRAME_ID.value]
        self.time_stamps = agent[0, EAgentProp.TIME_STAMP.value]
        self.cls = agent[0, EAgentProp.CLASS.value]

        # 提取历史数据
        self.hist_traj = np.column_stack([hist_x, hist_y]).astype(np.float32)
        
        return np.column_stack([hist_x, hist_vx, hist_y, hist_vy]).astype(np.float32)

    def IMM_pred(self, Z: np.ndarray, physical_model: PhysicalModel, 
                 model_set: EDatasource) -> List[np.ndarray]:
        """使用IMM进行轨迹预测
        
        Args:
            Z: 输入特征，shape (history_len, 4)
            physical_model: 物理模型库
            model_set: 行为意图
            
        Returns:
            预测步序列，长度为PREDICT_STEPS
        """
        params = physical_model.get_model_params(model_set.value)  # 转换为整数值
        models = self._create_models(params)
        
        imm = Imm(
            models,
            physical_model.get_model_trans(),
            params['P_trans'],
            params['U_prob']
        )
        
        self._init_model_state(models, Z)
        probs = self._filter_observations(imm, Z)
        
        # 使用最后的滤波状态进行预测
        final_states = [model.X.copy() for model in imm.models]
        return self.predict_steps(final_states, imm.models,
                                 physical_model.get_model_trans(), 
                                 probs[-1])
    

    def _create_models(self, params: Dict) -> List[KalmanFilter]:
        """根据参数创建卡尔曼滤波模型
        
        Args:
            params: 包含模型参数的字典
            
        Returns:
            卡尔曼滤波模型列表
        """
        models = []
        for model_name in self.MODEL_NAMES:
            model_params = params['models'][model_name]
            models.append(KalmanFilter(
                model_params['A'],
                model_params['H'],
                model_params['Q']
            ))
        return models

    def _init_model_state(self, models: List[KalmanFilter], Z: np.ndarray) -> None:
        """初始化模型状态，状态顺序为 [x, vx, ax, y, vy, ay]
        
        Args:
            models: 模型列表
            Z: (N, 4)的输入特征 [x, vx, y, vy]
        """
        # 确保Z是float32类型
        z0 = np.asarray(Z[0], dtype=np.float32)
        # z0[0]=x, z0[1]=vx, z0[2]=y, z0[3]=vy
        
        # CV模型: [x, vx, y, vy]
        models[0].X = np.array([[z0[0]], [z0[1]], [z0[2]], [z0[3]]], dtype=np.float32)
        
        # CA模型: [x, vx, ax, y, vy, ay]
        models[1].X = np.array([[z0[0]], [z0[1]], [0.0], [z0[2]], [z0[3]], [0.0]], 
                               dtype=np.float32)
        
        # CT模型 (ct1-ct5): [x, vx, y, vy, omega]
        for i in range(2, self.NUM_MODELS):
            models[i].X = np.array([[z0[0]], [z0[1]], [z0[2]], [z0[3]], [0.0]], 
                                   dtype=np.float32)

    
    def _filter_observations(self, imm: Imm, Z: np.ndarray) -> List[np.ndarray]:
        """对观测数据进行IMM滤波
        
        Args:
            imm: IMM实例
            Z: (N, 4)的观测特征 [x, vx, y, vy]
            
        Returns:
            各步骤的模型概率列表
        """
        probs = []
        for z in Z:
            z_float = np.asarray(z, dtype=np.float32).reshape(-1, 1)
            prob = np.copy(imm.filt(z_float))
            probs.append(prob)
        return probs
            
    def predict_steps(self, states: List[np.ndarray], models: List[KalmanFilter],
                     model_trans, model_prob: np.ndarray, 
                     steps: int = None) -> List[np.ndarray]:
        """预测未来steps步的状态
        
        Args:
            states: 当前状态列表（每个模型一个）
            models: 卡尔曼滤波模型列表
            model_trans: 模型转移矩阵
            model_prob: 模型概率分布
            steps: 预测步数，默认使用PREDICT_STEPS
            
        Returns:
            预测步序列
        """
        if steps is None:
            steps = self.PREDICT_STEPS
            
        adjust_prob = model_prob.copy()
        pred_step = []
        
        for _ in range(steps):
            # 更新各模型状态
            for i in range(len(states)):
                states[i] = np.dot(models[i].A, states[i])
            
            # 计算加权平均预测
            x_step = np.zeros(states[0].shape)
            for i in range(len(models)):
                x_step += np.dot(model_trans[0][i], states[i]) * adjust_prob[i]
            
            pred_step.append(x_step.copy())
            
        return pred_step

    
    def _convert_pred_to_traj(self, pred: List[np.ndarray]) -> np.ndarray:
        """将预测结果转换为轨迹数据
        
        状态顺序: CV[x,vx,y,vy], CA[x,vx,ax,y,vy,ay], CT[x,vx,y,vy,omega]
        提取 x（索引0）和 y 坐标
        
        Args:
            pred: 预测结果列表，每个元素是一步的预测状态
            
        Returns:
            shape为(steps, 2)的数组，列为[x, y]坐标
        """
        if not pred:
            return np.empty((0, 2))
        
        pred_x = [step[0, 0] for step in pred]
        pred_y = [step[2, 0] for step in pred]
        
        return np.column_stack([pred_x, pred_y])

    def _IMM_pred_2_MDN(self, left_pred: List[np.ndarray], 
                      right_pred: List[np.ndarray], 
                      straight_pred: List[np.ndarray]) -> Agent_IMM_output:
        """将IMM的预测结果转换为MDN的输入格式
        
        Args:
            left_pred: 左转意图的预测轨迹
            right_pred: 右转意图的预测轨迹
            straight_pred: 直行意图的预测轨迹
            
        Returns:
            Agent_IMM_output对象，包含三种意图的预测轨迹和概率
        """
        # 转换预测结果为轨迹坐标
        left_traj = self._convert_pred_to_traj(left_pred)
        right_traj = self._convert_pred_to_traj(right_pred)
        straight_traj = self._convert_pred_to_traj(straight_pred)
        
        # 保存概率值（从DEFAULT_PROBS中提取）
        probs = [
            self.DEFAULT_PROBS[EDatasource.LEFT_TURN],
            self.DEFAULT_PROBS[EDatasource.RIGHT_TURN],
            self.DEFAULT_PROBS[EDatasource.STRAIGHT],
        ]
        
        # 使用意图作为字典键，保存三种意图的预测轨迹
        pred_traj_by_intent = {
            self.DEFAULT_PROBS[EDatasource.LEFT_TURN]: left_traj,
            self.DEFAULT_PROBS[EDatasource.RIGHT_TURN]: right_traj,
            self.DEFAULT_PROBS[EDatasource.STRAIGHT]: straight_traj,
        }
        
        # 创建Agent_IMM_output对象，传递所有必需的参数
        agent_IMM_pred = Agent_IMM_output(
            agent_id=self.agent_id,
            frame_id=self.frame_id,  
            time_stamps=self.time_stamps,  
            hist_traj=self.hist_traj,
            pred_traj_by_intent=pred_traj_by_intent,
            prob=probs,
            pred_traj=[left_traj, right_traj, straight_traj]
        )
        
        return agent_IMM_pred

    def _MDN_data_process(self, data):
        """将MDN数据处理为模型输入格式
        
        Args:
            data: Agent_IMM_output对象
            
        Returns:
            模型输入格式的数据结构
        """
        # 这里可以根据MDN模型的输入要求进行数据处理和转换
        # 例如，提取历史轨迹、预测轨迹和概率等信息，并组织成模型需要的格式
        track_id = data.agent_id
        frame_id = data.frame_id
        cls = self.cls
        hist_traj = data.hist_traj.copy()
        cand_traj = data.pred_traj_by_intent.copy()  
        
        #类别编码 例[1, 0, 0] 行人 [0, 1, 0] 摩托/三轮 [0, 0, 1] 自行车
        is_pedestrian = 1 if cls == EAgentType.PEDESTRIAN.value else 0
        is_motorcycle_tricycle = 1 if cls == EAgentType.TRUCK.value else 0
        is_bicycle = 1 if cls == EAgentType.MOTORCYCLIST.value or cls == EAgentType.BICYCLE.value else 0
        # print(f"MDN数据处理: track_id={track_id}, frame_id={frame_id}, cls={cls}, is_pedestrian={is_pedestrian}, is_motorcycle_tricycle={is_motorcycle_tricycle}, is_bicycle={is_bicycle}")
        # 计算候选轨迹先验概率
        p_lr = self._history_only_prior(hist_traj, 0.1)

        # 重新构建 cand_traj 字典，使用先验概率作为键（格式化为字符串以避免浮点精度问题）
        cand_traj_new = {}
        index = 0
        for key, val in cand_traj.items():
            new_key = f'{p_lr[index]:.4f}'
            if new_key not in cand_traj_new:
                cand_traj_new[new_key] = val
            else:
                cand_traj_new[new_key + '0'] = val
            index += 1
        cand_traj = cand_traj_new

        #坐标归一化与旋转
        #以末帧为原点
        origin = hist_traj[-1].copy()
        dxy = hist_traj[0] - hist_traj[-1]
        #计算旋转角
        angle = -np.arctan2(dxy[1], dxy[0])
        is_rotate = True
        #进行变换
        hist_traj -= origin
        
        if is_rotate:
            hist_traj = self._rotate_xy_numpy(hist_traj, angle)
        #添加dx,dy速度特征
        hist_traj = self._cal_dxy(hist_traj)

        
        #将候选轨迹进行同样的坐标变换和特征计算
        for key, val in cand_traj.items():
            cand_traj[key] -= origin
            if is_rotate:
                cand_traj[key] = self._rotate_xy_numpy(cand_traj[key], angle)
            cand_traj[key] = self._cal_dxy(cand_traj[key])
        #权重归一化
        keys = list(cand_traj.keys())
        trajs = list(cand_traj.values())
        cand_key_array = np.array(keys, dtype=float) #.reshape(-1, 1)
        cand_key_array = cand_key_array/np.sum(cand_key_array)

        # cand_key_array.fill(1/3)
        cand_traj_array = np.stack(trajs, axis=0)

        #特征提取
        cand_feat = self._calculate_enhanced_candidate_features(hist_traj, cand_traj_array)

        
        #构建特征矩阵，包含类别特征和候选轨迹特征
        features = np.full((len(cand_key_array), 3), [is_pedestrian, is_motorcycle_tricycle, is_bicycle])
        cls_frames = np.concatenate((features, cand_feat), axis=1)
        self.W_frames      = self._as_batched(cand_key_array.astype(np.float32), target_ndim=2)  # [B, K]
        self.Cls_frames    = self._as_batched(cls_frames.astype(np.float32),    target_ndim=3)  # [B, K, C]
        self.X_hist_frames = self._as_batched(hist_traj.astype(np.float32),     target_ndim=3)  # [B, T, 4]
        self.X_cand_frames = self._as_batched(cand_traj_array.astype(np.float32), target_ndim=4) # [B, K, T, 4]

        # numbers = [char for char in raw_data[0] if char.isdigit()]
        # # number_sum = sum(int(num) for num in numbers)
        # number_sum = int(''.join(numbers[-5:]))
        # obj_id = raw_data[1].replace('P', '100')
        # seq_id = obj_id + str(number_sum)[-5:].rjust(5, '0') + str(raw_data[2]).rjust(3, '0')
        # seq_id = int(seq_id)
        # # if seq_id in self.seq_ids: # 1002106832?/
        # #     print(seq_id)
        # self.seq_ids.append(torch.tensor(seq_id))

        # print(f"MDN数据处理: track_id={track_id}, frame_id={frame_id}, cls={cls}")


    def _history_only_prior(self, history_xy, dt):
        """
        用 3s 历史估计 左/直/右 基线先验（logits）。
        思路：累计航向变化与平均偏航率 -> sigmoid 映射成左/右倾向；小偏转则增加直行倾向。
        返回 shape=(3,) 对应 [left, right，straight] 的 logits（未归一化）
        """
        # dx = np.diff(history_xy[:, 0])  # Δx between points
        # dy = np.diff(history_xy[:, 1])  # Δy between points
        v = np.diff(history_xy, axis=0)
        hd = np.arctan2(np.clip(v[:, 1], -1e6, 1e6), np.clip(v[:, 0], -1e6, 1e6))
        hd = np.concatenate([[hd[0]], hd], axis=0)
        hd = np.unwrap(hd)
        # spd = np.sqrt(np.sum(v*v, axis=-1))
        r = np.diff(hd, axis=0) / dt
        yr = np.concatenate(([r[0]], r))

        # dtheta = np.abs(np.diff(hd))
        # kappa = dtheta / spd
        # kappa = np.concatenate([[kappa[0]], kappa], axis=0)
        t = history_xy[-1] - history_xy[0]
        finally_hd = np.arctan2(t[1], t[0])
        yaw_total_vector = finally_hd - hd[0]
        yaw_total_vector = (yaw_total_vector + np.pi) % (2 * np.pi) - np.pi

        yaw_total = hd[-1] - hd[-10]
        alpha = 0  # 更信任向量方向
        yaw_total = alpha * yaw_total_vector + (1 - alpha) * yaw_total

        yaw_rate = np.mean(yr)

        # 将转向趋势映射为左右分量
        # 经验：阈值约 5~10 度(≈0.09~0.17rad) 开始有显著偏向
        scale = 6 # 越大越“尖锐”
        left_tendency = np.tanh(scale * max(yaw_total, 0.0) + 0.5*scale * max(yaw_rate, 0.0))
        right_tendency = np.tanh(scale * max(-yaw_total, 0.0) + 0.5*scale * max(-yaw_rate, 0.0))
        # 直行倾向：偏航越小越大
        straight_tendency = np.exp(- (abs(yaw_total) * 3.0))  # 小角度 -> 更大直行分数

        # 组成 logits（未softmax）
        base = np.array([
            left_tendency,
            right_tendency,
            straight_tendency
        ], dtype=np.float32)
        # base = base / np.sum(base)

        logits = np.array(base, dtype=np.float32)
        logits_max = np.max(logits)
        exp_logits = np.exp((logits - logits_max) / max(0.2, 1e-8))
        return exp_logits / np.sum(exp_logits)
    
    def _rotate_xy_numpy(self, traj: np.ndarray, phi: float, center=None) -> np.ndarray:
        """
        对 (T,2) 或 (N,T,2) 的数组进行逆时针旋转。
        - phi: 旋转角（弧度）
        - center: (cx, cy) 旋转中心；None 表示原点
        """
        arr = np.asarray(traj).copy()
        R = self._rotation_matrix(phi)
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
    
    def _rotation_matrix(self, phi: float) -> np.ndarray:
        c, s = np.cos(phi), np.sin(phi)
        return np.array([[c, -s], [s, c]], dtype=float)

    def _cal_dxy(self, points):
        dt = 0.1  # Time step (seconds)
        dx = np.diff(points[:, 0])  # Δx between points
        dy = np.diff(points[:, 1])  # Δy between points
        dx = np.concatenate((dx, [dx[-1]]))
        dy = np.concatenate((dy, [dy[-1]]))
        return np.column_stack((points, dx, dy))
    
    def _calculate_enhanced_candidate_features(self, hist_traj, cand_traj):
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
    
    def _MDN_predict(self, agent):
        """使用MDN进行轨迹预测"""
        # 这里可以根据MDN模型的输入要求，使用self.W_frames, self.Cls_frames, self.X_hist_frames, self.X_cand_frames等数据进行预测
        # 例如，构建输入张量，加载MDN模型，进行前向传播，得到预测结果等
        self.model.eval()
        with torch.no_grad():
            x_hist = self.X_hist_frames.to(self.device)
            x_cand = self.X_cand_frames.to(self.device)
            wi = self.W_frames.to(self.device)
            cls_feature = self.Cls_frames.to(self.device)

            pi, mu, sigma, logits = self.model.inference(x_hist, x_cand, cls_feature, wi)

            top1 = pi.argmax(dim=-1)  # [B]

            traj_pi = (pi.unsqueeze(-1).unsqueeze(-1) * mu).sum(dim=1)  # [B, T, 2]

            batch_size, K, T, _ = mu.shape
            for batch_id in range(batch_size):
                y_pred = {}
                pred_top = mu[batch_id].cpu().numpy()
                pi_top = pi[batch_id].cpu().numpy()
                
                for i in range(len(pi_top)):
                    str_score = f'{pi_top[i]:.6f}'
                    while str_score in y_pred:
                        str_score = str_score + '0'  # Append one more '0'
                    y_pred[str_score] = pred_top[i]

                y_pred['1'] = traj_pi[batch_id].cpu().numpy()
                hist_traj = self.hist_traj.copy()   
                origin = hist_traj[-1].copy()
                dxy = hist_traj[0] - hist_traj[-1]
                angle = np.arctan2(dxy[1], dxy[0])
                is_rotate = True
                for key, val in y_pred.items():
                        if is_rotate:
                            y_pred[key] = self._rotate_xy_numpy(y_pred[key], angle)
                        y_pred[key] += origin
                for k, traj in y_pred.items():
                    new_traj_global = self.rebuild_predicted_path_from_endpoint(
                        agent, traj[-1], num_frames=30
                    )
                    y_pred[k] = new_traj_global
                return y_pred

        



            
