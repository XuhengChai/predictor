import math
import os
import logging
import numpy as np
import pandas as pd

from collections import deque
from typing import Any
# from zf_data_preprocess import DataPreprocess
from .zf_common import EDatasource, EAgentProp, CommonParams, EMacro, EMacroProp
from .predictor.zf_predictor_base import BaseInputPropcess
from .ui.zf_vehicle_status import EIntegrationMethod, VehicleState


logger = logging.getLogger(__name__)

class OnlineInputPropcessLidar(BaseInputPropcess):
    def __init__(self, method):
        super().__init__()
        self.method = method
        self.use_ego_gps = (method == EDatasource.ONLINE_GPS)
        self.scenario_states: dict[str, VehicleState] = {}

    def _validate_frame_consistency(self, data):
        """Validate that a single frame batch is not silently mixed across keys."""
        scenario_values = {str(row.get(EMacroProp.SCENE_ID.value)) for row in data}
        frame_values = {int(row.get(EMacroProp.FRAME_ID.value)) for row in data}
        timestamp_values = {float(row.get(EMacroProp.TIME_STAMP.value)) for row in data}
        if len(scenario_values) != 1 or len(frame_values) != 1 or len(timestamp_values) != 1:
            logger.error(
                "Mixed lidar frame data detected: scenario_id=%s frame_id=%s time_stamp=%s",
                sorted(scenario_values),
                sorted(frame_values),
                sorted(timestamp_values),
            )
            raise ValueError("Mixed scenario_id/frame_id/time_stamp in a single lidar frame")
        scenario_id = next(iter(scenario_values))
        frame_id = next(iter(frame_values))
        time_stamp = next(iter(timestamp_values))
        return scenario_id, frame_id, time_stamp

    def update_abs_pos_with_obj(self, scene_id, frame_id, time_stamp, ego_dict, obj_dicts, use_ego_gps, change_origin=True):
        """Update ego/object absolute states and return the existing frame-dict format."""
        if scene_id not in self.scenario_states:
            logger.info("Initializing VehicleState for new scenario_id: %s", scene_id)
            self.scenario_states.clear()
            self.scenario_states[scene_id] = VehicleState()
        vehicle_state = self.scenario_states[scene_id]
        # vehicle_state = self.scenario_states.setdefault(scene_id, VehicleState())
        if use_ego_gps:
            ego_x, ego_y, ego_v, ego_theta, obj_results, origin_meta = vehicle_state.update_rel_obj_gps_ego(
                ego_dict,
                obj_dicts,
                return_origin_meta=True,
                change_origin=change_origin,
            )
        else:
            ego_x, ego_y, ego_v, ego_theta, obj_results, origin_meta = vehicle_state.update_abs_pos_with_obj(
                ego_dict,
                obj_dicts,
                return_origin_meta=True,
                ig_method=EIntegrationMethod.EULER,
                change_origin=change_origin,
            )
        ego_dict[EAgentProp.X] = ego_x
        ego_dict[EAgentProp.Y] = ego_y
        ego_dict[EAgentProp.VX] = ego_v * math.cos(ego_theta)
        ego_dict[EAgentProp.VY] = ego_v * math.sin(ego_theta)
        ego_dict[EAgentProp.YAW] = math.atan2(math.sin(ego_theta), math.cos(ego_theta))
        ego_dict[EAgentProp.V] = ego_v
        ego_dict[EAgentProp.OX] = ego_x
        ego_dict[EAgentProp.OY] = ego_y
        ego_dict[EAgentProp.OYAW] = ego_theta
        ego_dict[EAgentProp.IS_RESET] = bool(origin_meta.get("origin_reset", False))
        for obj_dict, obj_result in zip(obj_dicts, obj_results):
            _, abs_x, abs_y, abs_yaw, abs_vx, abs_vy = obj_result
            obj_dict[EAgentProp.X] = abs_x
            obj_dict[EAgentProp.Y] = abs_y
            obj_dict[EAgentProp.VX] = abs_vx
            obj_dict[EAgentProp.VY] = abs_vy
            obj_dict[EAgentProp.YR] = np.nan
            obj_dict[EAgentProp.YAW] = math.atan2(math.sin(abs_yaw), math.cos(abs_yaw))
            obj_dict[EAgentProp.V] = np.nan if np.isnan(abs_vx) or np.isnan(abs_vy) else math.sqrt(abs_vx ** 2 + abs_vy ** 2)
            obj_dict[EAgentProp.OX] = abs_x
            obj_dict[EAgentProp.OY] = abs_y
            obj_dict[EAgentProp.OYAW] = abs_yaw
            obj_dict[EAgentProp.IS_RESET] = bool(origin_meta.get("origin_reset", False))
        return {
            frame_id: {
                EMacro.EGO_DATA: ego_dict,
                EMacro.OBJ_DATA: obj_dicts,
                EMacro.SCENE_ID: scene_id,
                EMacro.ORIGIN_META: origin_meta,
            }
        }


    def process_input(self, data, change_origin=True):
        """Convert one lidar frame into the existing frame-dict format used by the predictor."""
        if not data:
            return {}
        scene_id, frame_id, time_stamp = self._validate_frame_consistency(data)
        ego_dict = self.dict.copy()
        ego_dict[EAgentProp.TIME_STAMP] = time_stamp
        ego_dict[EAgentProp.FRAME_ID] = frame_id
        ego_dict[EAgentProp.TRACK_ID] = "AV"
        ego_dict[EAgentProp.CLASS] = self.class_dict["vehicle"].value
        ego_dict[EAgentProp.OBJECT_CATEGORY] = -1
        ego_dict[EAgentProp.V] = float(data[0].get(EMacroProp.EGO_V.value, np.nan))
        ego_dict[EAgentProp.YR] = float(data[0].get(EMacroProp.EGO_YAWRATE.value, np.nan))
        if self.use_ego_gps:
            ego_dict[EAgentProp.X] = data[0].get(EMacroProp.EGO_X.value)
            ego_dict[EAgentProp.Y] = data[0].get(EMacroProp.EGO_Y.value)
            ego_dict[EAgentProp.YAW] = data[0].get(EMacroProp.EGO_HEADING_RAD.value)
        obj_dicts = []
        for row in data:
            agent_type = str(row.get(EMacroProp.AGENT_TYPE.value, "unknown"))
            if agent_type not in self.class_dict or row.get(EMacroProp.TRACK_ID.value) == "AV":
                continue
            obj_dict = self.dict.copy()
            obj_dict[EAgentProp.TIME_STAMP] = time_stamp
            obj_dict[EAgentProp.FRAME_ID] = frame_id
            obj_dict[EAgentProp.CLASS] = self.class_dict[agent_type].value
            obj_dict[EAgentProp.TRACK_ID] = str(row.get(EMacroProp.TRACK_ID.value))
            obj_dict[EAgentProp.OBJECT_CATEGORY] = 3
            obj_dict[EAgentProp.X] = float(row.get(EMacroProp.REL_X.value))
            obj_dict[EAgentProp.Y] = float(row.get(EMacroProp.REL_Y.value))
            obj_dict[EAgentProp.VX] = float(row.get(EMacroProp.REL_VX.value, np.nan))
            obj_dict[EAgentProp.VY] = float(row.get(EMacroProp.REL_VY.value, np.nan))
            obj_dicts.append(obj_dict)
        return self.update_abs_pos_with_obj(scene_id, frame_id, time_stamp, ego_dict, obj_dicts, self.use_ego_gps,change_origin)

class GPSInputPropcess(BaseInputPropcess):
    def __init__(self):
        super().__init__()
        self.method = EDatasource.OFFLINE_GPS

    def process_input(self, csv_path, change_origin=False):
        """
           将CSV数据转换为按frame_id组织的字典格式，模拟在线数据。
           CSV列应为：track_id, frame_id, agent_type, x, y, object_category。
           自车track_id为"AV"。
           返回字典：{frame_id: {'ego_dict': ego_dict, 'obj_dicts': [obj_dict1, ...]}}
           其中ego_dict和obj_dicts的键为EAgentProp枚举值。
           """
        if csv_path.endswith(".csv"):

            raw_data = pd.read_csv(csv_path)
        elif csv_path.endswith(".parquet"):
            raw_data = pd.read_parquet(csv_path)
        else:
            raise ValueError("Unsupported file format. Only .csv and .parquet are supported.")
        raw_data = raw_data[raw_data['agent_type'].isin(self.allowed_cls)]
        # TODO: grouped by scened_id
        grouped = raw_data.groupby('frame_id')
        frames_dict = {}
        for frame_id, group in grouped:
            # ego_data = group[group['track_id'] == 'AV']
            # obj_data = group[group['track_id'] != 'AV']
            # 构建obj_dicts列表
            self.dict[EAgentProp.TIME_STAMP] = int(frame_id)  # 假设frame_id为时间戳
            self.dict[EAgentProp.FRAME_ID] = int(frame_id)
            obj_dicts = []
            ego_dict = {}
            scene_id = group['scenario_id'].unique()[0]
            for _, row in group.iterrows():
                obj_dict = self.dict.copy()
                obj_dict[EAgentProp.X] = float(row['x'])
                obj_dict[EAgentProp.Y] = float(row['y'])
                obj_dict[EAgentProp.CLASS] = self.class_dict[row['agent_type']].value
                obj_dict[EAgentProp.TRACK_ID] = str(row['track_id'])
                obj_dict[EAgentProp.OBJECT_CATEGORY] = int(row['object_category'])
                if "frame_level" in row:
                    obj_dict[EAgentProp.LEVEL] = int(row['frame_level'])
                if row['track_id'] == 'AV':
                    ego_dict = obj_dict
                else:
                    obj_dicts.append(obj_dict)
            frames_dict[frame_id] = {EMacro.EGO_DATA: ego_dict, EMacro.OBJ_DATA: obj_dicts, EMacro.SCENE_ID: scene_id}
        return frames_dict

class RelInputPropcess(BaseInputPropcess):
    def __init__(self):
        super().__init__()
        self.method = EDatasource.OFFLINE_RELATIVE
        self.reset()
        # print("RelInputPropcess initialized.")

    def reset(self):
        self.scenario_states = {}
        self.origin_by_id = {}
        self.frame_meta_by_id = {}
        self.frame_meta_by_frame_key = {}
        self.saved_frame_packets_path = None

    @staticmethod
    def _safe_float(value, default=np.nan):
        if pd.isna(value):
            return default
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _normalize_angle(angle):
        if np.isnan(angle):
            return angle
        return math.atan2(math.sin(angle), math.cos(angle))

    def _read_input(self, csv_path):
        if csv_path.endswith(".csv"):
            return pd.read_csv(csv_path)
        if csv_path.endswith(".xlsx"):
            return pd.read_excel(csv_path)
        if csv_path.endswith(".parquet"):
            return pd.read_parquet(csv_path)
        raise ValueError("Unsupported file format. Only .csv and .parquet are supported.")

    @staticmethod
    def _safe_int(value, default=-1):
        if pd.isna(value):
            return default
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def _update_ego_dict_absolute(self, ego_dict, ego_x, ego_y, ego_yaw, ego_v):
        ego_dict[EAgentProp.X] = ego_x
        ego_dict[EAgentProp.Y] = ego_y
        ego_dict[EAgentProp.VX] = ego_v * math.cos(ego_yaw)
        ego_dict[EAgentProp.VY] = ego_v * math.sin(ego_yaw)
        ego_dict[EAgentProp.YAW] = self._normalize_angle(ego_yaw)
        ego_dict[EAgentProp.V] = ego_v
        ego_dict[EAgentProp.OX] = ego_x
        ego_dict[EAgentProp.OY] = ego_y
        ego_dict[EAgentProp.OYAW] = self._normalize_angle(ego_yaw)

    def _update_obj_dicts_absolute(self, obj_dicts, obj_results, is_reset):
        # valid_count = min(len(obj_dicts), len(obj_results))
        # for obj_dict, obj_result in zip(obj_dicts[:valid_count], obj_results[:valid_count]):
        for obj_dict, obj_result in zip(obj_dicts, obj_results):
            _, abs_x, abs_y, abs_yaw, abs_vx, abs_vy = obj_result
            obj_dict[EAgentProp.X] = abs_x
            obj_dict[EAgentProp.Y] = abs_y
            obj_dict[EAgentProp.VX] = abs_vx
            obj_dict[EAgentProp.VY] = abs_vy
            obj_dict[EAgentProp.YR] = np.nan
            obj_dict[EAgentProp.YAW] = self._normalize_angle(abs_yaw)
            obj_dict[EAgentProp.OX] = abs_x
            obj_dict[EAgentProp.OY] = abs_y
            obj_dict[EAgentProp.OYAW] = self._normalize_angle(abs_yaw)
            obj_dict[EAgentProp.IS_RESET] = is_reset
            if np.isnan(abs_vx) or np.isnan(abs_vy):
                obj_dict[EAgentProp.V] = np.nan
            else:
                obj_dict[EAgentProp.V] = math.sqrt(abs_vx ** 2 + abs_vy ** 2)
        return obj_dicts
        # return obj_dicts[:valid_count]

    def update_abs_pos_with_obj(self, scene_id, frame_id, ego_dict, obj_dicts):
        vehicle_state = self.scenario_states.setdefault(scene_id, VehicleState())
        ego_x, ego_y, ego_v, ego_theta, obj_results, origin_meta = vehicle_state.update_abs_pos_with_obj(
            ego_dict,
            obj_dicts,
            return_origin_meta=True,
            ig_method=EIntegrationMethod.EULER,
        )
        is_reset = bool(origin_meta.get("origin_reset", False))
        self._update_ego_dict_absolute(ego_dict, ego_x, ego_y, ego_theta, ego_v)
        ego_dict[EAgentProp.IS_RESET] = is_reset
        obj_dicts = self._update_obj_dicts_absolute(obj_dicts, obj_results, is_reset)
        origin_id = (scene_id, origin_meta.get('origin_ts'))
        self.origin_by_id[int(frame_id)] = origin_id
        self.frame_meta_by_frame_key[(str(scene_id), int(frame_id))] = origin_meta
        if origin_id not in self.frame_meta_by_id:
            self.frame_meta_by_id[origin_id] = origin_meta
        return {
            EMacro.EGO_DATA: ego_dict,
            EMacro.OBJ_DATA: obj_dicts,
            EMacro.SCENE_ID: scene_id,
            EMacro.ORIGIN_META: origin_meta,
        }

    def process_frame(self, scene_id, frame_id, group):
        self.dict[EAgentProp.TIME_STAMP] = float(group['time_stamp'].unique()[0])  # 假设frame_id为时间戳
        self.dict[EAgentProp.FRAME_ID] = int(frame_id)
        obj_dicts = []
        ego_dict = {}
        # scene_id = group['scenario_id'].unique()[0]
        for _, row in group.iterrows():
            obj_dict = self.dict.copy()
            obj_dict[EAgentProp.CLASS] = self.class_dict[row['agent_type']].value
            obj_dict[EAgentProp.TRACK_ID] = str(row['track_id'])
            obj_dict[EAgentProp.OBJECT_CATEGORY] = int(row['object_category'])
            if row['track_id'] == 'AV':
                obj_dict[EAgentProp.V] = float(row['ego_v'])
                obj_dict[EAgentProp.YR] = float(row['ego_yawrate'])
                ego_dict = obj_dict
            else:
                obj_dict[EAgentProp.X] = self._safe_float(row.get('rel_x'))
                obj_dict[EAgentProp.Y] = self._safe_float(row.get('rel_y'))
                if np.isnan(obj_dict[EAgentProp.X]) or np.isnan(obj_dict[EAgentProp.Y]):
                    continue
                obj_dict[EAgentProp.VX] = self._safe_float(row.get('rel_vx'))
                obj_dict[EAgentProp.VY] = self._safe_float(row.get('rel_vy'))
                obj_dicts.append(obj_dict)
        return self.update_abs_pos_with_obj(scene_id, frame_id, ego_dict, obj_dicts)

    def process_input(self, input_data, save_csv_flag=False):
        self.reset()
        raw_data = self._read_input(input_data)
        if raw_data.empty:
            return {}
        raw_data = raw_data[raw_data['agent_type'].isin(self.allowed_cls)]
        raw_data['track_id'] = raw_data['track_id'].astype(str)
        raw_data = raw_data.sort_values(
            by=['scenario_id', 'frame_id', 'track_id'],
            kind='mergesort'
        )
        grouped = raw_data.groupby(['scenario_id', 'frame_id'], sort=False)
        frames_dict = {}
        save_csv_flag = False
        if save_csv_flag:
            raw_data['ego_x_cal'] = np.nan
            raw_data['ego_y_cal'] = np.nan
            raw_data['ego_yaw_cal'] = np.nan
            raw_data['obj_x'] = np.nan
            raw_data['obj_y'] = np.nan
            raw_data['obj_yaw'] = np.nan

        for (scene_id, frame_id), group in grouped:
            frames_dict[frame_id] = self.process_frame(scene_id, frame_id, group)
            if save_csv_flag:
                ego_dict = frames_dict[frame_id][EMacro.EGO_DATA]
                ego_x = ego_dict[EAgentProp.X]
                ego_y = ego_dict[EAgentProp.Y]
                ego_yaw = ego_dict[EAgentProp.YAW]
                raw_data.loc[group.index, 'ego_x_cal'] = ego_x
                raw_data.loc[group.index, 'ego_y_cal'] = ego_y
                raw_data.loc[group.index, 'ego_yaw_cal'] = ego_yaw
                obj_by_id = {
                    str(obj[EAgentProp.TRACK_ID]): obj
                    for obj in frames_dict[frame_id][EMacro.OBJ_DATA]
                }
                for idx, row in group.iterrows():
                    track_id = str(row['track_id'])
                    if track_id == 'AV':
                        continue
                    obj = obj_by_id.get(track_id)
                    if obj is None:
                        continue
                    raw_data.loc[idx, 'obj_x'] = obj[EAgentProp.X]
                    raw_data.loc[idx, 'obj_y'] = obj[EAgentProp.Y]
                    raw_data.loc[idx, 'obj_yaw'] = obj[EAgentProp.YAW]
        if save_csv_flag:
            save_dir = os.path.dirname(input_data).replace('dataset', 'dataset_online')
            os.makedirs(save_dir, exist_ok=True)
            csv_name = (
                f"{os.path.splitext(os.path.basename(input_data))[0]}"
                "_abs_pos.csv"
            )
            output_csv = os.path.join(save_dir, csv_name)
            raw_data.to_csv(output_csv, index=False)
        return frames_dict


class InputDataRecorder:
    def __init__(self, data_source: EDatasource = EDatasource.ONLINE, his_max_length=30):
        if his_max_length is not np.nan:
            self.his_max_length = his_max_length
        else:
            self.his_max_length = CommonParams.obj_history * CommonParams.obj_frequency
        self.data_source = data_source
        self.NUM_AGENT_COLS = len(EAgentProp)
        self._ego_history = deque(maxlen=his_max_length)
        self._obj_history = {}   # key: track_id, value: deque(maxlen=30)
        self._obj_last_frame = {}  # 记录每个对象最后被观察到的帧号
        self._current_rel_origin_meta = None

    def clear(self):
        self._ego_history.clear()
        self._obj_history.clear()
        self._obj_last_frame.clear()
        self._current_rel_origin_meta = None

    def _cleanup_old_objects(self, current_frame_id):
        to_delete = []
        for tid, last_frame in self._obj_last_frame.items():
            # 如果对象超过his_max_length帧没有出现，则删除
            if current_frame_id - last_frame >= self.his_max_length:
                to_delete.append(tid)
        for tid in to_delete:
            if tid in self._obj_history:
                del self._obj_history[tid]
            if tid in self._obj_last_frame:
                del self._obj_last_frame[tid]

    def update(self, ego_dict, obj_dicts, data_source: EDatasource = EDatasource.OFFLINE_GPS, frame_meta=None):
        """
        ego_dict: dict of ego data for this frame
        obj_dicts: list of dicts, each dict is one object for this frame
        """
        # update ego
        self.data_source = data_source
        ego_props = np.array(list(ego_dict.values()), dtype=object)
        self._ego_history.append(ego_props)
        current_frame_id = ego_dict[EAgentProp.FRAME_ID]
        self._cleanup_old_objects(current_frame_id)
        self._set_relative_origin_meta(frame_meta)
        self.update_gps_data(ego_dict, obj_dicts)
        return self.get_agents()

    def get_agents(self):
        # if len(self._ego_history) < self.his_max_length:
        #     return np.empty((0,self.NUM_AGENT_COLS))
        ego_data = np.array(list(self._ego_history))
        if self.data_source != EDatasource.OFFLINE_GPS:
            ego_data = self._transform_history_to_current_origin(ego_data)
        for tid, his in self._obj_history.items():
            if his[-1][EAgentProp.FRAME_ID] < self._ego_history[-1][EAgentProp.FRAME_ID]:
                continue
            obj_data = np.array(list(his))
            if self.data_source != EDatasource.OFFLINE_GPS:
                obj_data = self._transform_history_to_current_origin(obj_data)
            ego_data = np.append(ego_data, obj_data, axis=0)
        return ego_data

    def update_gps_data(self, ego_dict, obj_dicts):
        current_frame_id = ego_dict[EAgentProp.FRAME_ID]
        ego_x, ego_y = ego_dict[EAgentProp.X], ego_dict[EAgentProp.Y]
        # update objects
        for obj in obj_dicts:
            tid = obj[EAgentProp.TRACK_ID]
            obj_x, obj_y = obj[EAgentProp.X], obj[EAgentProp.Y]
            dis = np.sqrt((obj_x - ego_x) ** 2 + (obj_y - ego_y) ** 2)
            # if dis > CommonParams.ego_range:
            #     continue
            if tid not in self._obj_history:
                self._obj_history[tid] = deque(maxlen=self.his_max_length)
            agent_prop = np.array(list(obj.values()), dtype=object)
            self._obj_history[tid].append(agent_prop)

            self._obj_last_frame[tid] = current_frame_id

    @staticmethod
    def _safe_float_value(value, default=np.nan):
        try:
            if pd.isna(value):
                return default
        except TypeError:
            pass
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _normalize_angle(angle):
        if np.isnan(angle):
            return angle
        return math.atan2(math.sin(angle), math.cos(angle))

    def _set_relative_origin_meta(self, frame_meta):
        if frame_meta:
            self._current_rel_origin_meta = frame_meta

    def _transform_history_to_current_origin(self, agent_history):
        if len(agent_history) == 0 or not self._current_rel_origin_meta:
            return agent_history

        T_inv = self._current_rel_origin_meta.get('T_inv')
        origin_theta = self._current_rel_origin_meta.get('origin_theta')
        if T_inv is None or origin_theta is None:
            return agent_history
        transformed_history = agent_history.copy()

        for idx in range(len(transformed_history)):
            row = transformed_history[idx].copy()
            x_world = self._safe_float_value(row[EAgentProp.X], np.nan)
            y_world = self._safe_float_value(row[EAgentProp.Y], np.nan)
            if not (np.isnan(x_world) or np.isnan(y_world)):
                row[EAgentProp.X] = T_inv[0, 0] * x_world + T_inv[0, 1] * y_world + T_inv[0, 2]
                row[EAgentProp.Y] = T_inv[1, 0] * x_world + T_inv[1, 1] * y_world + T_inv[1, 2]

            vx_world = self._safe_float_value(row[EAgentProp.VX], np.nan)
            vy_world = self._safe_float_value(row[EAgentProp.VY], np.nan)
            if not (np.isnan(vx_world) or np.isnan(vy_world)):
                row[EAgentProp.VX] = T_inv[0, 0] * vx_world + T_inv[0, 1] * vy_world
                row[EAgentProp.VY] = T_inv[1, 0] * vx_world + T_inv[1, 1] * vy_world

            yaw_world = self._safe_float_value(row[EAgentProp.YAW], np.nan)
            if not np.isnan(yaw_world):
                row[EAgentProp.YAW] = self._normalize_angle(yaw_world - origin_theta)

            transformed_history[idx] = row
        return transformed_history


class FeaturePreprocess:
    def __init__(self, data_source=EDatasource.ONLINE, his_max_length=30):
        self.data_source = data_source
        self.data_recorder = InputDataRecorder(data_source, his_max_length)
        self.NUM_AGENT_COLS = len(EAgentProp)
        self._origin_by_id = {}
        self._frame_meta_by_id = {}
        self.input_process = {
            EDatasource.OFFLINE_GPS: GPSInputPropcess(),
            EDatasource.OFFLINE_RELATIVE: RelInputPropcess(),
            EDatasource.ONLINE_GPS: OnlineInputPropcessLidar(self.data_source),
            EDatasource.ONLINE_NO_GPS: OnlineInputPropcessLidar(self.data_source),
        }
        self.his_max_length = his_max_length
        self.agents = np.empty((0, len(EAgentProp)))

    def get_frames_dict(self, csv_file_path): # for new scene id
        self.data_recorder.clear()
        return self.get_frame_dict(csv_file_path)

    def get_frame_dict(self, input_param, change_origin=True):
        # for offline data, params is csv_file_path; for online data, params is a list of dicts
        processor = self.input_process[self.data_source]
        frames_dict = processor.process_input(input_param, change_origin)
        self._origin_by_id = getattr(processor, 'origin_by_id', {})
        self._frame_meta_by_id = getattr(processor, 'frame_meta_by_id', {})
        return frames_dict

    def update(self, ego_dict, obj_dicts, data_source: EDatasource, frame_meta=None):
        """
        Called once per frame.
        """
        if frame_meta is None and data_source == EDatasource.OFFLINE_RELATIVE:
            frame_id = int(ego_dict[EAgentProp.FRAME_ID])
            origin_id = self._origin_by_id.get(frame_id)
            frame_meta = self._frame_meta_by_id.get(origin_id)
        self.agents = self.data_recorder.update(ego_dict, obj_dicts, data_source, frame_meta=frame_meta)
        if CommonParams.DEBUG and len(self.agents):
            df = pd.DataFrame(self.agents, columns=EAgentProp.names())
        return self.agents

    def _convert_record_to_row(self, rec):
        """
        Convert a single dict → 1×NUM_COLS row. Missing attrs = np.nan.
        """
        row = np.zeros(self.NUM_AGENT_COLS, dtype=object)
        for key in EAgentProp:
            if key in rec:
                row[key] = rec[key]
            else:
                row[key] = np.nan
        # row = np.array(list(rec.values()))
        return row

    def get_agents(self):
        # convert_to_np_input
        return self.agents

if __name__ == '__main__':
    # data_queue = deque(maxlen=5)
    # data_queue.append(np.array([0, 1, 2, 3]))
    # data_queue.append(np.array([0.1, 1.1, 2.1, 3.1]))
    # data_queue.append(np.array([0.2, 1.2, 2.2, 3.2]))
    # data_queue.append(np.array([0.3, 1.3, 2.3, 3.3]))
    # print(data_queue)
    # print(type(data_queue))
    # data = np.array(list(data_queue))
    # print(data)
    fp = FeaturePreprocess(EDatasource.ONLINE)
    # agents = fp.convert_agents_to_np()
    # print(agents.shape)
