
import math
import numpy as np
from enum import Enum
# try:
from zf_traj_prediction.zf_common import EAgentProp, CommonParams
# except ImportError:
#     try:
#         from ..zf_common import EAgentProp, CommonParams
#     except ImportError:
#         EAgentProp = None
#         CommonParams = None


def _prop(name, default_value):
    if EAgentProp is None:
        return default_value
    return getattr(EAgentProp, name)


DEFAULT_DIMENSIONS_SET = (
    CommonParams.dimensions_set
    if CommonParams is not None
    else {
        'pedestrian': (0.8, 0.5, 1.8),
        'bicycle': (1.6, 0.3, 1.865),
        'cyclist': (1.6, 0.3, 1.865),
        'car': (4.075, 1.668, 1.474),
        'vehicle': (4.075, 1.668, 1.474),
        'motorcyclist': (1.75, 0.5, 1.0),
        'bus': (9.6, 2.5, 3.45),
        'truck': (7.2, 2.3, 2.7),
        'unknown': (0.5, 0.5, 0.5),
        4: (0.5, 0.5, 0.5),
        8: (0.5, 0.5, 0.5),
        9: (0.5, 0.5, 0.5),
        10: (0.5, 0.5, 0.5),
    }
)

# 添加枚举定义
class EVehProps(Enum):
    TIME = _prop("TIME_STAMP", "time_stamp")
    VEL = _prop("V", "speed")
    YAW_RATE = _prop("YR", "yaw_rate")
    ACCX = "LongitudinalAcceleration"
    ACCY = "LateralAcceleration"

class EObjProps(Enum):
    REL_LONG_POS = _prop("X", "x")
    REL_LAT_POS = _prop("Y", "y")
    REL_LONG_VEL = _prop("VX", "vx")
    REL_LAT_VEL = _prop("VY", "vy")
    HEADING = _prop("YAW", "yaw")
    TRACK_ID = _prop("TRACK_ID", "track_id")

class EIntegrationMethod(Enum):
    EULER = "euler"
    TRAPEZOID = "trapezoid"

class VehicleState:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # 朝向（弧度）
        self.v = 0.0
        self.yr = 0.0
        self.last_time = None
        self.abs_traj = []  # 存储绝对轨迹
        self.obj_abs_trajs = {}  # 存储各目标的绝对轨迹 {obj_id: [(x, y, heading, timestamp), ...]}
        # key = straight_start_time，value = {"T": 局部→全局矩阵, "T_inv": 全局→局部矩阵}
        self.segment_transforms = {}
        self.reset_stack_tm = []  # [(timestamp, transform_matrix, inv_transform_matrix), ...]
        self.straight_start_time = None
        self.last_sw_angle = 0.0
        self.last_origin_meta = None
        self.straight_start_theta = None
        self.gps_origin_pose = None
        self.last_global_theta = None

    def _create_transform_matrix(self, x, y, theta):
        """创建变换矩阵 (3x3齐次变换矩阵)"""
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        trans = np.array([
            [cos_t, -sin_t, x],
            [sin_t, cos_t, y],
            [0, 0, 1]
        ]) #local -> global
        inv_trans = np.array([
            [cos_t, sin_t, -cos_t * x - sin_t * y],
            [-sin_t, cos_t, sin_t * x - cos_t * y],
            [0, 0, 1]
        ])
        return trans, inv_trans

    def _apply_transform(self, point, transform_matrix):
        """应用变换矩阵到点 (x, y)"""
        p = np.array([point[0], point[1], 1.0])
        p_transformed = transform_matrix @ p
        return p_transformed[0], p_transformed[1]


    def check_and_reset(self, frame, sw_angle_key='sw_angle', change_origin = True):
        """
            need_reset: bool
            T: local -> global
            T_inv: global -> local
        """
        if not change_origin and len(self.reset_stack_tm):
            return False
        current_time = float(frame[EVehProps.TIME.value])
        if sw_angle_key in frame:
            sw_angle = abs(float(frame[sw_angle_key]))
        else:
            sw_angle = abs(float(frame.get(EVehProps.YAW_RATE.value, 0.0))) * 0.1
        # if velocity < 0.5, also as straight driving
        is_straight = sw_angle < 0.33 or abs(float(frame[EVehProps.VEL.value])) < 0.5
        heading_delta_threshold = math.radians(5.0)
        # 确保第一帧一定有一个原点
        # if True:
        if not self.reset_stack_tm:
            T, T_inv = self._create_transform_matrix(self.x, self.y, self.theta)
            self.reset_stack_tm.append(current_time)
            self.segment_transforms[current_time] = {
                "T": T,
                "T_inv": T_inv,
                "origin_x": self.x,
                "origin_y": self.y,
                "origin_theta": self.theta
            }
            self.last_origin_meta = self.get_current_origin_meta(current_time, origin_reset=True)
            self.straight_start_time = current_time if is_straight else None
            self.straight_start_theta = self.theta if is_straight else None
            return True

        if not is_straight:
            # 关键修正：直行被打断，必须清空
            self.straight_start_time = None
            self.straight_start_theta = None
            self.last_origin_meta = self.get_current_origin_meta(current_time, origin_reset=False)
            return False
        else:
            if self.straight_start_time is None:
                self.straight_start_time = current_time
                self.straight_start_theta = self.theta
                self.last_origin_meta = self.get_current_origin_meta(current_time, origin_reset=False)
                return False

        if self.straight_start_theta is not None:
            heading_delta = math.atan2(
                math.sin(self.theta - self.straight_start_theta),
                math.cos(self.theta - self.straight_start_theta)
            )
            if abs(heading_delta) > heading_delta_threshold:
                self.straight_start_time = current_time
                self.straight_start_theta = self.theta
                return False

        # 连续直行超过阈值才创建新原点
        reset_interval_sec = 30.0
        if current_time - self.straight_start_time >= reset_interval_sec:
            T, T_inv = self._create_transform_matrix(self.x, self.y, self.theta)
            self.reset_stack_tm.append(current_time)
            self.segment_transforms[current_time] = {
                "T": T,
                "T_inv": T_inv,
                "origin_x": self.x,
                "origin_y": self.y,
                "origin_theta": self.theta
            }
            self.straight_start_time = current_time
            self.straight_start_theta = self.theta
            self.last_origin_meta = self.get_current_origin_meta(current_time, origin_reset=True)
            # print("change origin at time:", current_time, self.last_origin_meta)
            return True

        self.last_origin_meta = self.get_current_origin_meta(current_time, origin_reset=False)
        return False

    def get_current_origin_meta(self, current_time=None, origin_reset=False):
        if not self.reset_stack_tm:
            return {
                "origin_reset": origin_reset,
                "origin_ts": None,
                "T": None,
                "T_inv": None,
                "origin_x": self.x,
                "origin_y": self.y,
                "origin_theta": self.theta,
                "current_time": current_time,
            }

        origin_ts = self.reset_stack_tm[-1]
        seg_info = self.segment_transforms.get(origin_ts, {})
        return {
            "origin_reset": origin_reset,
            "origin_ts": origin_ts,
            "T": seg_info.get("T"),
            "T_inv": seg_info.get("T_inv"),
            "origin_x": seg_info.get("origin_x", self.x),
            "origin_y": seg_info.get("origin_y", self.y),
            "origin_theta": seg_info.get("origin_theta", self.theta),
            "current_time": current_time,
        }


    def get_transformed_trajectory(self, trajectory_points, target_time=None):
        """获取经过变换后的轨迹点

        Args:
            trajectory_points: [(timestamp, x, y, heading), ...]
            target_time: 目标时间点，如果为None则返回最新变换后的轨迹
        """
        pass

    def transform_to_world(self, x_r, y_r, x_abs, y_abs, yaw):
        """将相对坐标转换到世界坐标系"""
        x_w = x_abs + x_r * math.cos(yaw) - y_r * math.sin(yaw)
        y_w = y_abs + x_r * math.sin(yaw) + y_r * math.cos(yaw)
        return x_w, y_w

    def update_abs_pos_with_obj(self, frame, objs=None, return_origin_meta=True,
                                ig_method: EIntegrationMethod = EIntegrationMethod.TRAPEZOID
                                , change_origin=True):
        """
        更新车辆状态并计算目标绝对位置及绝对朝向
        """
        # 处理第一帧
        if not self.last_time:
            if np.isnan(frame[EVehProps.VEL.value]):
                result = (0, 0, self.v, 0, np.nan, np.nan, np.nan)
                if return_origin_meta:
                    return (*result, self.get_current_origin_meta(frame[EVehProps.TIME.value], origin_reset=False))
                return result

            self.last_time = frame[EVehProps.TIME.value]
            # self.straight_start_time = self.last_time
            origin_reset = self.check_and_reset(frame, change_origin=change_origin)
            self.v = frame[EVehProps.VEL.value]
            self.yr = frame[EVehProps.YAW_RATE.value]

            # 记录自车初始位置
            self.abs_traj.append((self.x, self.y, self.theta, self.last_time))

            # 计算目标绝对位置
            results = []
            if objs:
                for obj in objs:
                    rel_long = obj.get(EObjProps.REL_LONG_POS.value, np.nan)
                    rel_lat = obj.get(EObjProps.REL_LAT_POS.value, np.nan)
                    if not (np.isnan(rel_long) or np.isnan(rel_lat)):
                        obj_abs_x, obj_abs_y = self.transform_to_world(
                            rel_long, rel_lat, self.x, self.y, self.theta
                        )
                        # 计算目标绝对朝向
                        v_rx = obj.get(EObjProps.REL_LONG_VEL.value, np.nan)
                        v_ry = obj.get(EObjProps.REL_LAT_VEL.value, np.nan)

                        # 计算目标绝对速度
                        if not (np.isnan(v_rx) or np.isnan(v_ry)) and (abs(v_rx) > 1e-6 or abs(v_ry) > 1e-6):
                            psi_rel = math.atan2(v_ry, v_rx)
                            obj_heading = self.theta + psi_rel

                            # 计算绝对速度：自车速度 + 相对速度旋转到世界坐标系
                            v_ego_x = self.v * math.cos(self.theta)
                            v_ego_y = self.v * math.sin(self.theta)
                            abs_vx = v_ego_x + v_rx * math.cos(self.theta) - v_ry * math.sin(self.theta)
                            abs_vy = v_ego_y + v_rx * math.sin(self.theta) + v_ry * math.cos(self.theta)
                        else:
                            obj_heading = self.theta
                            abs_vx = np.nan
                            abs_vy = np.nan

                        obj_id = obj.get(EObjProps.TRACK_ID.value, 'unknown')
                        results.append((obj_id, obj_abs_x, obj_abs_y, obj_heading, abs_vx, abs_vy))
                        # 存储目标轨迹
                        if obj_id not in self.obj_abs_trajs:
                            self.obj_abs_trajs[obj_id] = []
                        self.obj_abs_trajs[obj_id].append(
                            (obj_abs_x, obj_abs_y, obj_heading, abs_vx, abs_vy, self.last_time)
                        )
            result = (self.x, self.y, self.v, self.theta, results)
            if return_origin_meta:
                return (*result, self.get_current_origin_meta(self.last_time, origin_reset=origin_reset))
            return result

        # 处理后续帧
        if np.isnan(frame[EVehProps.VEL.value]):
            frame[EVehProps.VEL.value] = self.v

        dt = frame[EVehProps.TIME.value] - self.last_time
        if ig_method == EIntegrationMethod.EULER:
            average_yr = frame[EVehProps.YAW_RATE.value]
            average_vel = frame[EVehProps.VEL.value]
            # average_yr = self.yr
            # average_vel = self.v
        else:
            average_yr = (self.yr + frame[EVehProps.YAW_RATE.value]) / 2
            average_vel = (self.v + frame[EVehProps.VEL.value]) / 2
        self.theta = self.theta + average_yr * dt
        v_x = average_vel * math.cos(self.theta)
        v_y = average_vel * math.sin(self.theta)
        self.x += v_x * dt
        self.y += v_y * dt

        # 记录自车轨迹
        self.abs_traj.append((self.x, self.y, self.theta, frame[EVehProps.TIME.value]))

        # 计算目标绝对位置和朝向
        results = []
        if objs:
            for obj in objs:
                rel_long = obj.get(EObjProps.REL_LONG_POS.value, np.nan)
                rel_lat = obj.get(EObjProps.REL_LAT_POS.value, np.nan)
                if not (np.isnan(rel_long) or np.isnan(rel_lat)):
                    obj_abs_x, obj_abs_y = self.transform_to_world(
                        rel_long, rel_lat, self.x, self.y, self.theta
                    )
                    # 计算目标绝对朝向
                    v_rx = obj.get(EObjProps.REL_LONG_VEL.value, np.nan)
                    v_ry = obj.get(EObjProps.REL_LAT_VEL.value, np.nan)

                    # 计算目标绝对速度
                    if not (np.isnan(v_rx) or np.isnan(v_ry)) and (abs(v_rx) > 1e-6 or abs(v_ry) > 1e-6):
                        psi_rel = math.atan2(v_ry, v_rx)
                        obj_heading = self.theta + psi_rel

                        # 计算绝对速度：自车速度 + 相对速度旋转到世界坐标系
                        v_ego_x = self.v * math.cos(self.theta)
                        v_ego_y = self.v * math.sin(self.theta)
                        abs_vx = v_ego_x + v_rx * math.cos(self.theta) - v_ry * math.sin(self.theta)
                        abs_vy = v_ego_y + v_rx * math.sin(self.theta) + v_ry * math.cos(self.theta)
                    else:
                        obj_heading = self.theta
                        abs_vx = np.nan
                        abs_vy = np.nan

                    obj_id = obj.get(EObjProps.TRACK_ID.value, 'unknown')
                    results.append((obj_id, obj_abs_x, obj_abs_y, obj_heading, abs_vx, abs_vy))
                    # 存储目标轨迹
                    if obj_id not in self.obj_abs_trajs:
                        self.obj_abs_trajs[obj_id] = []
                    self.obj_abs_trajs[obj_id].append(
                        (obj_abs_x, obj_abs_y, obj_heading, abs_vx, abs_vy, frame[EVehProps.TIME.value])
                    )

        # 更新状态
        self.v = frame[EVehProps.VEL.value]
        self.yr = frame[EVehProps.YAW_RATE.value]
        self.last_time = frame[EVehProps.TIME.value]
        origin_reset = self.check_and_reset(frame, change_origin=change_origin)
        result = (self.x, self.y, self.v, self.theta, results)
        if return_origin_meta:
            return (*result, self.get_current_origin_meta(self.last_time, origin_reset=origin_reset))
        return result

    def update_rel_obj_gps_ego(self, frame, objs=None, return_origin_meta=True, change_origin=False):
        """Update relative objects using gps ego pose while keeping object world conversion unchanged."""
        current_time = float(frame[EVehProps.TIME.value])
        if self.last_time is not None and current_time > self.last_time and self.last_global_theta is not None:
            frame[EVehProps.YAW_RATE.value] = math.atan2(
                math.sin(self.theta - self.last_global_theta),
                math.cos(self.theta - self.last_global_theta),
            ) / (current_time - self.last_time)
        self.last_time = current_time
        self.v = frame[EVehProps.VEL.value]
        self.yr = frame[EVehProps.YAW_RATE.value]
        ego_x_global = float(frame[EObjProps.REL_LONG_POS.value])
        ego_y_global = float(frame[EObjProps.REL_LAT_POS.value])
        ego_theta_global = float(frame[EObjProps.HEADING.value])
        self.x = float(frame[EObjProps.REL_LONG_POS.value])
        self.y = float(frame[EObjProps.REL_LAT_POS.value])
        self.theta = float(frame[EObjProps.HEADING.value])
        self.check_and_reset(frame, change_origin=change_origin)
        T_inv = self.last_origin_meta.get("T_inv")
        origin_theta = self.last_origin_meta.get("origin_theta")
        d_x = ego_x_global - self.last_origin_meta.get("origin_x")
        d_y = ego_y_global - self.last_origin_meta.get("origin_y")
        self.x = T_inv[0, 0] * d_x + T_inv[0, 1] * d_y
        self.y = T_inv[1, 0] * d_x + T_inv[1, 1] * d_y
        # self.x = T_inv[0, 0] * ego_x_global + T_inv[0, 1] * ego_y_global + T_inv[0, 2]
        # self.y = T_inv[1, 0] * ego_x_global + T_inv[1, 1] * ego_y_global + T_inv[1, 2]
        self.theta = math.atan2(math.sin(ego_theta_global - origin_theta), math.cos(ego_theta_global - origin_theta))
        self.last_global_theta = float(frame[EObjProps.HEADING.value])
        self.abs_traj.append((self.x, self.y, self.theta, current_time))

        results = []
        if objs:
            for obj in objs:
                rel_long = obj.get(EObjProps.REL_LONG_POS.value, np.nan)
                rel_lat = obj.get(EObjProps.REL_LAT_POS.value, np.nan)
                if not (np.isnan(rel_long) or np.isnan(rel_lat)):
                    obj_abs_x, obj_abs_y = self.transform_to_world(
                        rel_long, rel_lat, self.x, self.y, self.theta
                    )
                    v_rx = obj.get(EObjProps.REL_LONG_VEL.value, np.nan)
                    v_ry = obj.get(EObjProps.REL_LAT_VEL.value, np.nan)
                    if not (np.isnan(v_rx) or np.isnan(v_ry)) and (abs(v_rx) > 1e-6 or abs(v_ry) > 1e-6):
                        psi_rel = math.atan2(v_ry, v_rx)
                        obj_heading = self.theta + psi_rel
                        v_ego_x = self.v * math.cos(self.theta)
                        v_ego_y = self.v * math.sin(self.theta)
                        abs_vx = v_ego_x + v_rx * math.cos(self.theta) - v_ry * math.sin(self.theta)
                        abs_vy = v_ego_y + v_rx * math.sin(self.theta) + v_ry * math.cos(self.theta)
                    else:
                        obj_heading = self.theta
                        abs_vx = np.nan
                        abs_vy = np.nan

                    obj_id = obj.get(EObjProps.TRACK_ID.value, 'unknown')
                    results.append((obj_id, obj_abs_x, obj_abs_y, obj_heading, abs_vx, abs_vy))
                    if obj_id not in self.obj_abs_trajs:
                        self.obj_abs_trajs[obj_id] = []
                    self.obj_abs_trajs[obj_id].append(
                        (obj_abs_x, obj_abs_y, obj_heading, abs_vx, abs_vy, current_time)
                    )

        result = (self.x, self.y, self.v, self.theta, results)
        if return_origin_meta:
            return (*result, self.last_origin_meta)
        return result
