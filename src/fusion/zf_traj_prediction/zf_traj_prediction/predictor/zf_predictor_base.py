from abc import ABC, abstractmethod
from zf_traj_prediction.zf_common import EAgentType, EAgentProp, EPredictorName
import numpy as np


class BasePredictor(ABC):
    def __init__(self):
        super().__init__()
        self.method = EPredictorName.BASE

    @abstractmethod
    def predict_traj(self, agent, surroundings): #-> Ptraj class
        agent_feature = self.process_traj(agent, surroundings)
        return np.empty((0, 2))

    @abstractmethod
    def process_traj(self, agent, surroundings):
        return agent

    def inverse_calc(self, vx_avg: float, vy_avg: float, psi0: float, horizon: float = 3.0):
        theta = np.arctan2(vy_avg, vx_avg)
        delta_psi = 2 * (theta - psi0)
        # delta_psi = (delta_psi + np.pi) % (2 * np.pi) - np.pi
        speed_norm = np.sqrt(vx_avg ** 2 + vy_avg ** 2)
        if np.abs(delta_psi) < 1e-6:
            sinc_factor = 1.0
        else:
            sinc_factor = 2 * np.sin(delta_psi / 2) / delta_psi
        v_avg = speed_norm / max(abs(sinc_factor), 1e-6)
        omega = delta_psi / horizon if horizon != 0 else 0.0
        return float(v_avg), float(omega)

    def rebuild_predicted_path_from_endpoint(self, agent, end_point, num_frames=30):
        """
        Rebuild trajectory using kinematic model starting from agent state.
        Uses only endpoint, similar to your first approach.
        """
        # Extract current state
        last_xy = agent[-1, :2]
        vx, vy = agent[-1, 2:4]
        psi0 = np.arctan2(vy, vx)
        cur_x, cur_y = last_xy
        rel_disp = end_point - last_xy
        vx_rel, vy_rel = rel_disp * 10 / num_frames  # Assuming dt=0.1
        v, yaw_rate = self.inverse_calc(vx_rel, vy_rel, psi0, horizon=num_frames * 0.1)
        # Current speed from agent
        cur_v = np.sqrt(vx ** 2 + vy ** 2)
        # Integration
        dt = 0.1
        a = (v - cur_v) / 6  # Same acceleration profile as reference
        trajectory = np.zeros((num_frames, 2))
        x, y, vel, yaw = cur_x, cur_y, cur_v, psi0
        for i in range(num_frames):
            if i < 7:
                vel = vel + a * dt
            else:
                vel = v
            yaw = yaw + yaw_rate * dt
            x += vel * np.cos(yaw) * dt
            y += vel * np.sin(yaw) * dt
            trajectory[i] = [x, y]
        # Endpoint correction (optional, but helps with accuracy)
        endpoint_error = end_point - trajectory[-1]
        start_idx = 2 * num_frames // 3
        for i in range(start_idx, num_frames):
            alpha = (i - start_idx) / (num_frames - start_idx) + 0.1
            trajectory[i] += alpha * endpoint_error
        return trajectory

# TODO Litao
# class outputClass

class TwoWheelerPredictor(BasePredictor):
    def __init__(self):
        super().__init__()
        self.method = "two_wheeler"  # 两轮车预测器

    def predict_traj(self, agent, surroundings):
        """两轮车轨迹预测实现"""
        # 这里实现两轮车的具体预测逻辑
        agent_feature = self.process_traj(agent, surroundings)
        return np.array([[0, 0], [2, 2], [4, 4]])  # 示例

    def process_traj(self, agent, surroundings):
        """处理两轮车轨迹"""
        print(f"Processing two-wheeler trajectory with method: {self.method}")
        # 具体的处理逻辑
        return agent

class BaseInputPropcess(ABC):
    def __init__(self):
        super().__init__()
        self.method = "base"
        self.allowed_cls = ['pedestrian', 'cyclist', 'bicycle', 'motorcyclist', 'vehicle', 'bus']
        self.dict = {}
        for attr in EAgentProp:
            self.dict[attr] = np.nan
        self.class_dict = {
            "pedestrian": EAgentType.PEDESTRIAN,
            "bicycle": EAgentType.BICYCLE,
            "cyclist": EAgentType.CYCLIST,
            "motorcyclist": EAgentType.MOTORCYCLIST,
            "vehicle": EAgentType.VEHICLE,
            "bus": EAgentType.BUS,
            "truck": EAgentType.TRUCK,
        }

    @abstractmethod
    def process_input(self, input_data):
        """
            return {frame_id: {'ego_dict': ego_dict, 'obj_dicts': [obj_dict1, ...]}}
        """
        frames_dict = {}
        frames_dict[0] = {'ego_dict': {}, 'obj_dicts': []}
        return frames_dict

