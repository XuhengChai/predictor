from .zf_predictor_base import BasePredictor
from zf_traj_prediction.zf_common import EAgentProp
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import pickle

class KalmanFilter:
    def __init__(self, A, H, R):
        self.A = A  # nx × nx State Transition Matrix: F
        # self.B = B  # nx × nu Control Matrix
        self.B = np.eye(A.shape[0])
        self.H = H  # nz × nx Observation Matrix
        self.Q = np.eye(A.shape[0])  # nx × nx Process Noise Covariance 过程噪声协方差
        self.R = R  # nz × nz Measurement Covariance 测量噪声

        self.U = np.zeros((self.B.shape[1], 1))  # nu × 1
        self.X = np.zeros((A.shape[0], 1))  # nx × 1
        self.X_pre = self.X
        self.P = np.zeros(A.shape)    # nx × nx
        self.P_pre = self.P    # nx × nx 估计协方差 Estimate Covariance

    # def __init__(self, A, H):
    #     self.A = A  # nx × nx State Transition Matrix: F
    #     self.B = np.eye(A.shape[0])    # nx × nu Control Matrix
    #     self.H = H  # nz × nx Observation Matrix
    #     self.Q = np.eye(A.shape[0])  # nx × nx Process Noise Covariance 过程噪声协方差
    #     self.R = np.eye(H.shape[0])  # nz × nz Measurement Covariance 测量噪声
    #
    #     self.U = np.zeros((self.B.shape[1], 1))  # nu × 1
    #     self.X = np.zeros((A.shape[0], 1))  # nx × 1
    #     self.X_pre = self.X    # nx × 1
    #     self.P = np.zeros(A.shape)  # nx × nx
    #     self.P_pre = self.P    # nx × nx

    def set_Q(self, sigma_a, dt=1.0):
        self.Q = self._cal_Q(sigma_a, dt)
    def set_init_X(self, x_state, p_init):
        self.X = x_state
        self.P = p_init
    def set_R(self,r):
        self.R=r
    def filt(self, Z):  # nz × 1
        self._predict()
        self._update(Z.reshape(-1, 1))
        return self.X

    def predict(self, x_status):
        x_new = np.dot(self.A, x_status.reshape(-1, 1)) + np.dot(self.B, self.U)
        return x_new

    def _predict(self):
        self.X_pre = np.dot(self.A, self.X.reshape(-1, 1)) + np.dot(self.B, self.U)
        # self.X_pre = self.X
        self.P_pre = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.X_pre

    def _update(self, Z):
        K = np.dot(np.dot(self.P_pre, self.H.T),
                   np.linalg.inv(np.dot(np.dot(self.H, self.P_pre), self.H.T) +\
                                 self.R))
        self.X = self.X_pre + np.dot(K, Z - np.dot(self.H, self.X_pre))
        # self.P = self.P_pre - np.dot(np.dot(K, self.H), self.P_pre)
        KH = np.dot(K, self.H)
        I_ = np.eye(self.P_pre.shape[0])
        self.P = np.dot(np.dot(I_ - KH, self.P_pre), (I_ - KH).T) + np.dot(np.dot(K, self.R), K.T)

    def __cal_Q(self, sigma_a, dt, is_continue=True):
        if is_continue:
            Q_c = sigma_a * np.array([
                [dt ** 5 / 20, dt ** 4 / 8, dt ** 3 / 6],
                [dt ** 4 / 8, dt ** 3 / 3, dt ** 2 / 2],
                [dt ** 3 / 6, dt ** 2 / 2, dt]
            ])
            return Q_c
        Q = sigma_a * np.array([
            [0.25 * dt ** 4, 0.5 * dt ** 3, 0.5 * dt ** 2],
            [0.5 * dt ** 3, dt ** 2, dt],
            [0.5 * dt ** 2, dt, 1]
        ])
        return Q

    def cal_ADE(self, pred, target):
        pred = pred[:, :2]
        target = target[:, :2]
        assert pred.shape == target.shape  # [batch_size, len,dim] [16,30,2]
        # temp = np.sum((pred - target) ** 2)
        tmp = np.sqrt(np.sum((pred - target) ** 2, axis=1))  # [batch_size, len]
        ade = np.mean(tmp, axis=0, keepdims=True)  # [batch_size, 1]
        return float(ade)

    def cal_FDE(self, pred, target):
        # print(pred.shape, target.shape)
        pred = pred[:, :2]
        target = target[:, :2]
        assert pred.shape == target.shape  # batch_size, 30, 2
        pred = pred[-1, :]  # [batch_size, dim]
        target = target[-1, :]
        fde = np.sqrt(np.sum((pred - target) ** 2, axis=0, keepdims=True))  # [batch_size, 1]
        return float(fde)

    def cal_angle(self, pred, target):
        pred = pred[:, :2]
        target = target[:, :2]
        assert pred.shape == target.shape  # 确保形状一致
        # 构造向量：首尾点之差
        pred_vec = pred[-1, :] - pred[0, :]
        target_vec = target[-1, :] - target[0, :]
        # 计算点积和模长
        dot_product = np.dot(pred_vec, target_vec)
        norm_pred = np.linalg.norm(pred_vec)
        norm_target = np.linalg.norm(target_vec)
        # 防止除以0
        if norm_pred == 0 or norm_target == 0:
            return np.nan
        # 计算夹角的余弦值
        cos_theta = dot_product / (norm_pred * norm_target)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)  # 防止数值误差
        # 转换为角度
        angle_rad = np.arccos(cos_theta)
        angle_deg = np.degrees(angle_rad)
        return angle_deg

class CVPredictor(KalmanFilter, BasePredictor):
    def __init__(self):
        self.dt = 0.1
        A = np.array([
            [1.0, self.dt, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, self.dt],
            [0.0, 0.0, 0.0, 1.0]
        ])
        H = np.eye(4)
        # Q = 0.05563749 * np.eye(4)
        # R = 0.01 * np.eye(4)
        Q = 0.05563749 * np.eye(4)
        R = 0.5 * np.eye(4)
        super().__init__(A, H, R)
        self.Q = Q
        self.P = np.array([
            [0.1, 0, 0.0, 0.0],
            [0.0, 0.05, 0.0, 0.0],
            [0.0, 0.0, 0.1, 0],
            [0.0, 0.0, 0.0, 0.05]
        ], dtype=np.float32)
        self.history = deque(maxlen=30)

    def predict_traj(self, agent, surroundings):
        agent_feature = self.process_traj(agent, surroundings)
        self.filter_history(agent_feature, len(agent_feature))
        pred_traj = self.predict_future()
        return {"1": pred_traj}

    def process_traj(self, agent, surroundings):
        result = agent[:, [EAgentProp.X, EAgentProp.VX, EAgentProp.Y, EAgentProp.VY]]
        return result

    def filter_agent(self, agent, steps=30):
        result = agent[:, [EAgentProp.X, EAgentProp.VX, EAgentProp.Y, EAgentProp.VY]]
        self.X = result[0]
        self.history.append([float(self.X[0]), float(self.X[2])])
        for i in range(1, steps):
            temp_x = self.filt(result[i])
            agent[i, EAgentProp.X] = float(temp_x[0])
            agent[i, EAgentProp.VX] = float(temp_x[1])
            agent[i, EAgentProp.Y] = float(temp_x[2])
            agent[i, EAgentProp.VY] = float(temp_x[3])
        return agent


    def filter_history(self, his, steps=30):
        self.X = his[0]
        self.history.append([float(self.X[0]), float(self.X[2])])
        for i in range(1, steps):
            temp_x = self.filt(his[i])
            self.history.append([float(temp_x[0]), float(temp_x[2])])
        return self.history

    def predict_future(self, steps=30): # truth
        predicted_states = []
        x_new = self.X.copy()
        for id in range(steps):
            x_new = self.predict(x_new)
            predicted_states.append([float(x_new[0]), float(x_new[2])])
        return np.array(predicted_states)
        # ade = self.cal_ADE(np.array(predicted_states), np.array(truth))
        # fde = self.cal_FDE(np.array(predicted_states), np.array(truth))
        # angle = self.cal_angle(np.array(predicted_states), np.array(truth))
        # # print("error ", cc)
        # # print(self.x.copy())
        # history_array = np.array(self.history)
        # return np.array(predicted_states), history_array, ade, fde, angle

class CAPredictor(KalmanFilter, BasePredictor):
    def __init__(self):
        self.dt = 0.1
        A = np.array([
                    [1.0, self.dt, 0.5 * self.dt**2, 0.0, 0.0, 0.0],
                    [0.0, 1.0, self.dt, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0, self.dt, 0.5 * self.dt**2],
                    [0.0, 0.0, 0.0, 0.0, 1.0, self.dt],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                ])
        H = np.array([
                        [1., 0., 0., 0., 0., 0.],
                        [0., 1., 0., 0., 0., 0.],
                        [0., 0., 0., 1., 0., 0.],
                        [0., 0., 0., 0., 1., 0.],
                    ])
        Q = 0.08999768 * np.eye(6)
        R = 0.8 * np.eye(4)
        # H = np.eye(6)
        # R = 0.01 * np.eye(6)
        super().__init__(A, H, R)
        self.Q = Q
        self.P = np.array([
                    [0.1, 0, 0, 0.0, 0.0, 0.0],
                    [0.0, 0.05, 0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.001, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.1, 0, 0],
                    [0.0, 0.0, 0.0, 0.0, 0.05, 0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.001],
                ], dtype=np.float32)
        self.history = deque(maxlen=30)

    def predict_traj(self, agent, surroundings):
        agent_feature = self.process_traj(agent, surroundings)
        self.filter_history(agent_feature, len(agent_feature))
        pred_traj = self.predict_future()
        return {"1": pred_traj}

    def process_traj(self, agent, surroundings):
        result = agent[:, [EAgentProp.X, EAgentProp.VX, EAgentProp.Y, EAgentProp.VY]]
        return result

    def filter_history(self, his, steps=30):
        # his: 6*1 x vx ax y vy ay
        self.X = np.array([his[0][0],his[0][1],0, his[0][2],his[0][3], 0])
        # self.X = his[0]
        # self.history.append([float(self.X[0]), float(self.X[3])])
        # for i in range(1, steps):
        #     temp_x = self.filt(his[i])
        #     self.history.append([float(temp_x[0]), float(temp_x[3])])
        self.history.append(self.X.reshape(-1))
        for i in range(1, steps):
            temp_x = self.filt(his[i])
            # self.history.append([float(temp_x[0]), float(temp_x[1])])
            self.history.append(temp_x.reshape(-1))

    def predict_future(self, steps=30):
        # his: 6*1 x vx ax y vy ay
        predicted_states = []
        x_new = self.X.copy()
        for id in range(steps):
            x_new = self.predict(x_new)
            predicted_states.append([float(x_new[0]), float(x_new[3])])
        return np.array(predicted_states)

        # ade = self.cal_ADE(np.array(predicted_states), np.array(truth))
        # fde = self.cal_FDE(np.array(predicted_states), np.array(truth))
        # angle = self.cal_angle(np.array(predicted_states), np.array(truth))
        # # print("error ", cc)
        # # print(self.x.copy())
        # history_array = np.array(self.history)
        # return np.array(predicted_states), history_array, ade, fde, angle

class KalmanCAExtend(KalmanFilter, BasePredictor):
    def __init__(self):
        A = np.eye(6) # [x, y, v, yaw, yaw_rate, a]
        H = np.array([
                        [1., 0., 0., 0., 0., 0.],
                        [0., 1., 0., 0., 0., 0.],
                        [0., 0., 1., 0., 0., 0.],
                        # [0., 0., 0., 1., 0., 0.],
                    ])
        Q = 0.08999768 * np.eye(6)
        # R = 0.01 * np.eye(3)
        R = np.diag([0.01, 0.01, 0.2])
        # H = np.eye(6)
        # R = 0.01 * np.eye(6)
        self.std_acc = 0.05
        self.std_yaw_acc = 0.1
        self.dt = 0.1
        super().__init__(A, H, R)
        self.Q = Q
        self.P = np.array([
                    [0.1, 0, 0, 0.0, 0.0, 0.0],
                    [0.0, 0.1, 0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.05, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.05, 0, 0],
                    [0.0, 0.0, 0.0, 0.0, 0, 0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.001],
                ], dtype=np.float32)
        self.history = deque(maxlen=30)
        self.history_deque = deque(maxlen=3)

    def predict_traj(self, agent, surroundings):
        agent_feature = self.process_traj(agent, surroundings)
        self.filter_history(agent_feature, len(agent_feature))
        pred_traj = self.predict_future()
        return pred_traj

    def process_traj(self, agent, surroundings):
        return agent[:, [EAgentProp.X, EAgentProp.Y, EAgentProp.V, EAgentProp.YAW, EAgentProp.YR]]

    def _predict(self):
        x_pre, jacobian_a, q_new = self._predict_x(np.asarray(self.X).reshape(-1))
        self.X_pre = x_pre.reshape(-1, 1)
        self.P_pre = np.dot(np.dot(jacobian_a, self.P), jacobian_a.T) + q_new
        return self.X_pre

    def _update(self, Z):
        measurement = np.asarray(Z).reshape(-1, 1)
        innovation = measurement - np.dot(self.H, self.X_pre)
        innovation_cov = np.dot(np.dot(self.H, self.P_pre), self.H.T) + self.R
        K = np.dot(np.dot(self.P_pre, self.H.T), np.linalg.inv(innovation_cov))
        x_updated = self.X_pre + np.dot(K, innovation)
        kh = np.dot(K, self.H)
        identity = np.eye(self.P_pre.shape[0])
        self.P = np.dot(np.dot(identity - kh, self.P_pre), (identity - kh).T) + np.dot(np.dot(K, self.R), K.T)
        self._update_x(x_updated.reshape(-1))

    def _predict_x(self, x_status): # [x, y, v, yaw, yaw_rate, a]
        v = x_status[2]
        heading = x_status[3]
        heading_rate = x_status[4]
        a = x_status[5]
        dt = self.dt
        x_new = np.array([
                        x_status[0] + v * np.cos(heading) * dt +
                        1/2 * a * np.cos(heading) * dt * dt,
                        x_status[1] + v * np.sin(heading) * dt +
                        1/2 * a * np.sin(heading) * dt * dt,
                        v + a * dt,
                        heading,
                        heading_rate,
                        a])
        J_A02 = dt * np.cos(heading)
        J_A03 = - (dt * v + 1/2 * a * dt * dt) * np.sin(heading)
        J_A05 = 1/2 * dt * dt * np.cos(heading)
        J_A12 = dt * np.sin(heading)
        J_A13 = (dt * v + 1/2 * a * dt * dt) * np.cos(heading)
        J_A15 = 1/2 * dt * dt * np.sin(heading)
        J_new = np.array([[1, 0, J_A02, J_A03, 0, J_A05],
                             [0, 1, J_A12, J_A13, 0, J_A15],
                             [0, 0, 1, 0, 0, dt],
                             [0, 0, 0, 1, 0, 0],
                             [0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 1],
                             ])
        Q_v = np.diag([self.std_acc ** 2, self.std_yaw_acc ** 2])
        # Q_v = np.diag([(self.std_acc/10) ** 2, (self.std_yaw_acc  * 4) ** 2])
        # Q_v = np.diag([ 0.05** 2, 0.1 ** 2])
        temp = np.zeros([6, 2])
        temp[0, 0] = dt * dt * dt * np.cos(heading) / 6
        temp[1, 0] = dt * dt * dt * np.sin(heading) / 6
        temp[2, 0] = dt * dt * 0.5
        temp[3, 1] = 0.5 * dt * dt
        temp[4, 1] = 0
        temp[5, 0] = dt
        Q_new = np.dot(np.dot(temp, Q_v), temp.T)
        return x_new, J_new, Q_new

    def _update_x(self, x_new): # [x, y, v, yaw, yaw_rate, a]
        if len(self.history_deque) < 3:
            vx = (x_new[0] - self.X[0])/self.dt
            vy = (x_new[1] - self.X[1])/self.dt
            a = (x_new[2] - self.X[2])/self.dt
            v = np.sqrt(vx ** 2 + vy ** 2)
            yaw = np.arctan2(vy, vx)
            self.X = x_new
            # self.X[2] = v
            self.X[3] = yaw
            self.X[4] = 0
            # self.X[5] = a
        else:
            self.X = x_new
        self.history_deque.append(self.X.copy())
        v, yaw, yaw_rate, a = self._compute_kalman_state()
        # self.X[2] = v
        self.X[3] = yaw
        # self.X[5] = a

    def filter_history(self, his, steps=30):
        # his: 6*1 [x, y, v, yaw, yaw_rate, a]
        self.history.clear()
        self.X = np.array([his[0][0],his[0][1],his[0][2],his[0][3], 0, 0])
        # self.X = his[0]
        self.history_deque.clear()
        self.history_deque.append(self.X.copy())
        self.history.append(self.X)
        for i in range(1, steps):
            temp_x = self.filt(his[i, :3])
            # self.history.append([float(temp_x[0]), float(temp_x[1])])
            self.history.append(temp_x)

    def predict_future(self, truth=None, steps=30):
        predicted_states = []
        predicted = []
        x_new = np.asarray(self.X).reshape(-1).copy()
        for id in range(steps):
            x_new, _, _ = self._predict_x(x_new)
            predicted_states.append([float(x_new[0]), float(x_new[1])])
            predicted.append(x_new.copy())

        if truth is None:
            return {'1': np.array(predicted_states)}

        ade = self.cal_ADE(np.array(predicted_states), np.array(truth))
        fde = self.cal_FDE(np.array(predicted_states), np.array(truth))
        angle = self.cal_angle(np.array(predicted_states), np.array(truth))
        # print("error ", cc)
        # print(self.x.copy())
        history_array = np.array(self.history)[:, :2]
        return np.array(predicted_states), history_array, ade, fde, angle

    def _compute_kalman_state(self):
        if len(self.history_deque) < 3:
            return self.X[2:]
        dt = self.dt
        pos = np.array(self.history_deque)
        x = pos[-3:, 0]
        y = pos[-3:, 1]
        n = len(x)
        dx = np.diff(x)
        vx = np.zeros(n)
        vx[0] = dx[0] / dt
        vx[-1] = dx[-1] / dt
        vx[1:-1] = (dx[:-1] + dx[1:]) / (2 * dt)
        dy = np.diff(y)
        vy = np.zeros(n)
        vy[0] = dy[0] / dt
        vy[-1] = dy[-1] / dt
        vy[1:-1] = (dy[:-1] + dy[1:]) / (2 * dt)
        ax = np.diff(vx)/ dt
        ay = np.diff(vy)/ dt
        yaw = np.arctan2(vy[-1], vx[-1])
        v = np.sqrt(vx[-1] ** 2 + vy[-1] ** 2)
        a = np.sqrt(ax[-1] ** 2 + ay[-1] ** 2)
        yaw_rate = np.where(v < 0.01,
                            0,
                            (vx[-1] * ay[-1] - vy[-1] * ax[-1]) / (vx[-1] ** 2 + vy[-1] ** 2))
        return v, yaw, yaw_rate, a

if __name__ == "__main__":
    a = CVPredictor()