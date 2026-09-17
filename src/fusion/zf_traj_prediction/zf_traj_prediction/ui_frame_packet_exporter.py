import os
import csv
from typing import Any
import json
import numpy as np
import pandas as pd

from .zf_common import CommonParams, EAgentProp, EMacro
class OnlineLidarCsvWriter:
    FIELDNAMES = [
        "scenario_id", "track_id", "frame_id",
        "rel_x", "rel_y", "rel_vx", "rel_vy",
        "agent_type", "object_category", "frame_category","obj_score",
        "ego_v", "ego_yawrate", "time_stamp", "SteerWheelAngle",
        "x", "y", "yaw_rad",
        "ego_x", "ego_y", "ego_heading_rad",
        "OX", "OY", "OYAW",
        "gps_ego_x", "gps_ego_y", "gps_ego_heading_rad", "IS_RESET",
        "hist_traj", "pred_traj", "ptrajs_dict",
    ]
    PRED_TRACK_ID_INDEX = 1
    PRED_HIST_TRAJ_INDEX = 4
    PRED_TRAJ_INDEX = 5
    PRED_PTRAJS_DICT_INDEX = 8
    def __init__(self, output_path: str):
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        print("-----output_path------", output_path)
        self.file = open(
            output_path,
            "w",
            newline="",
            encoding="utf-8",
        )
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=self.FIELDNAMES,
            extrasaction="ignore",
        )
        self.writer.writeheader()
        self.file.flush()

    @staticmethod
    def _csv_value(v):
        if v is None:
            return ""
        return v.item() if isinstance(v, np.generic) else v

    @staticmethod
    def _traj_to_json(traj):
        if traj is None:
            return ""
        return json.dumps(
            np.asarray(traj).tolist(),
            ensure_ascii=False,
            separators=(",", ":"),
        )

    @staticmethod
    def _pred_to_json(pred_traj):
        if pred_traj is None:
            return ""
        return json.dumps(
            {
                str(prob): np.asarray(traj).tolist()
                for prob, traj in pred_traj.items()
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )

    @staticmethod
    def _prediction_value_to_xy(prediction_value):
        if prediction_value is None:
            return None
        if isinstance(prediction_value, dict):
            prediction_value = prediction_value.get("1")
        prediction_array = np.asarray(prediction_value, dtype=float)
        if (
            prediction_array.ndim != 2
            or prediction_array.shape[0] == 0
            or prediction_array.shape[1] < 2
        ):
            return None
        return prediction_array[:, :2].tolist()

    @classmethod
    def _ptrajs_to_json(cls, ptrajs_dict):
        if not isinstance(ptrajs_dict, dict):
            return ""
        normalized = {
            str(sensor_idx): trajectory
            for sensor_idx, prediction_value in ptrajs_dict.items()
            for trajectory in [cls._prediction_value_to_xy(prediction_value)]
            if trajectory is not None
        }
        return json.dumps(normalized, ensure_ascii=False, separators=(",", ":")) if normalized else ""

    def write_frame(self, frame_rows, agents, frame_preds):
        pred_lookup = {
            str(pred[self.PRED_TRACK_ID_INDEX]): {
                "hist_traj": self._traj_to_json(
                    pred[self.PRED_HIST_TRAJ_INDEX]
                ),
                "pred_traj": self._pred_to_json(
                    pred[self.PRED_TRAJ_INDEX]
                ),
            }
            for pred in frame_preds
            if len(pred) > self.PRED_TRAJ_INDEX
        }
        agent_lookup = {}
        if isinstance(agents, np.ndarray) and agents.ndim == 2:
            for track_id in np.unique(agents[:, EAgentProp.TRACK_ID]):
                agent_rows = agents[agents[:, EAgentProp.TRACK_ID] == track_id]
                if len(agent_rows):
                    agent_lookup[str(track_id)] = agent_rows[-1]
        ego_agent = agent_lookup.get("AV")
        common_cols = (
            "scenario_id", "frame_id",
            "rel_x", "rel_y", "rel_vx", "rel_vy",
            "agent_type",
            # "object_category","frame_category",
             "ego_v",
            "ego_yawrate", "time_stamp",
            "SteerWheelAngle",
        )
        has_ego = False
        for raw_row in frame_rows:
            row = {k: "" for k in self.FIELDNAMES}
            track_id = str(raw_row.get("track_id", ""))
            if track_id == "AV":
                has_ego = True
            for col in common_cols:
                row[col] = self._csv_value(raw_row.get(col))
            row.update({
                "track_id": track_id,
                "obj_score": self._csv_value(raw_row.get("obj_score")),
                "gps_ego_x": self._csv_value(raw_row.get("ego_x")),
                "gps_ego_y": self._csv_value(raw_row.get("ego_y")),
                "gps_ego_heading_rad": self._csv_value(
                    raw_row.get("ego_heading_rad")
                ),
            })
            if track_id == "AV":
                row["agent_type"] = "vehicle"
            agent = agent_lookup.get(track_id)
            if agent is not None:
                row["object_category"] = self._csv_value(agent[EAgentProp.OBJECT_CATEGORY])
                row["level"] = self._csv_value(agent[EAgentProp.LEVEL])
                row["x"] = self._csv_value(agent[EAgentProp.X])
                row["y"] = self._csv_value(agent[EAgentProp.Y])
                row["yaw_rad"] = self._csv_value(agent[EAgentProp.YAW])
                row["OX"] = self._csv_value(agent[EAgentProp.OX])
                row["OY"] = self._csv_value(agent[EAgentProp.OY])
                row["OYAW"] = self._csv_value(agent[EAgentProp.OYAW])
                row["IS_RESET"] = self._csv_value(agent[EAgentProp.IS_RESET])
            if ego_agent is not None:
                row["ego_x"] = self._csv_value(ego_agent[EAgentProp.X])
                row["ego_y"] = self._csv_value(ego_agent[EAgentProp.Y])
                row["ego_heading_rad"] = self._csv_value(ego_agent[EAgentProp.YAW])
            pred = pred_lookup.get(track_id)
            if pred:
                row["hist_traj"] = pred["hist_traj"]
                row["pred_traj"] = pred["pred_traj"]
            frame_pred = next(
                (
                    prediction
                    for prediction in frame_preds
                    if str(prediction[self.PRED_TRACK_ID_INDEX]) == track_id
                    and len(prediction) > self.PRED_PTRAJS_DICT_INDEX
                ),
                None,
            )
            if frame_pred is not None:
                row["ptrajs_dict"] = self._ptrajs_to_json(
                    frame_pred[self.PRED_PTRAJS_DICT_INDEX]
                )
            self.writer.writerow(row)
        raw_ego = frame_rows[0] if frame_rows else {}
        if not has_ego:
            row = {k: "" for k in self.FIELDNAMES}
            for col in common_cols:
                row[col] = self._csv_value(raw_ego.get(col))
            row["track_id"] = "AV"
            row["rel_x"] = 0
            row["rel_y"] = 0
            row["rel_vx"] = 0
            row["rel_vy"] = 0
            row["obj_score"] = 0
            row["object_category"] = -1
            row["level"] = -1
            row["agent_type"] = "vehicle"
            row["x"] = ego_agent[EAgentProp.X]
            row["y"] = ego_agent[EAgentProp.Y]
            row["yaw_rad"] = ego_agent[EAgentProp.YAW]
            row["OX"] = ego_agent[EAgentProp.OX]
            row["OY"] = ego_agent[EAgentProp.OY]
            row["OYAW"] = ego_agent[EAgentProp.OYAW]
            row["IS_RESET"] = self._csv_value(ego_agent[EAgentProp.IS_RESET])
            row["ego_x"] = self._csv_value(raw_ego.get("ego_x"))
            row["ego_y"] = self._csv_value(raw_ego.get("ego_y"))
            row["ego_heading_rad"] = self._csv_value(raw_ego.get("ego_heading_rad"))
            row["gps_ego_x"] = self._csv_value(raw_ego.get("ego_x"))
            row["gps_ego_y"] = self._csv_value(raw_ego.get("ego_y"))
            row["gps_ego_heading_rad"] = self._csv_value(raw_ego.get("ego_heading_rad"))
            self.writer.writerow(row)
        self.file.flush()

    def close(self):
        if not self.file.closed:
            self.file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class UIFramePacketExporter:
    ALLOWED_CLASSES = ["pedestrian", "cyclist", "bicycle", "motorcyclist", "vehicle", "bus"]
    PRIMARY_PREDICTION_KEY = "1"

    @staticmethod
    def _safe_json_loads(value):
        if isinstance(value, str):
            value = value.strip()
            if not value:
                return None
            try:
                return json.loads(value)
            except json.JSONDecodeError:
                return None
        return value

    @classmethod
    def _normalize_ptrajs_dict(cls, value):
        value = cls._safe_json_loads(value)
        if not isinstance(value, dict):
            return {}
        normalized = {}
        for sensor_idx, pred_value in value.items():
            pred_traj = cls._normalize_prediction_value(pred_value)
            if pred_traj is None:
                continue
            normalized[str(sensor_idx)] = pred_traj
        return normalized

    @classmethod
    def _normalize_metrics_dict(cls, value):
        value = cls._safe_json_loads(value)
        if not isinstance(value, dict):
            return {}
        normalized = {}
        for sensor_idx, metric_value in value.items():
            normalized[str(sensor_idx)] = cls._safe_float(metric_value)
        return normalized

    @staticmethod
    def _dump_json(value):
        if value in (None, ""):
            return ""
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))

    def export(self, input_path, prediction_source=None, save_flag=True):
        raw_data = self._prepare_raw_data(input_path, prediction_source)
        if raw_data.empty:
            return [] if not save_flag else None

        if not save_flag:
            self._build_frame_packets(raw_data)
            return self._save_frame_packets(self._build_frame_packets(raw_data), input_path, save_flag)
        else:
            return self._save_raw_dataframe(raw_data, input_path, save_flag)

    @staticmethod
    def _read_input(input_path):
        if input_path.endswith(".csv"):
            return pd.read_csv(input_path)
        if input_path.endswith(".xlsx"):
            return pd.read_excel(input_path)
        if input_path.endswith(".parquet"):
            return pd.read_parquet(input_path)
        raise ValueError("Unsupported file format. Only .csv, .xlsx and .parquet are supported.")

    @staticmethod
    def _safe_float(value, default=np.nan):
        if pd.isna(value):
            return default
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _safe_int(value, default=-1):
        if pd.isna(value):
            return default
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _normalize_prediction_value(value):
        # if isinstance(value, dict):
        #     if not value:
        #         return None
        #     best_key = max(value, key=lambda key: float(key))
        #     value = value[best_key]
        # pred_array = np.asarray(value, dtype=float)
        if isinstance(value, np.ndarray):
            return value.tolist()
        pred_array = value['1']
        if pred_array.ndim != 2 or pred_array.shape[1] < 2 or pred_array.shape[0] == 0:
            return None
        return pred_array[:, :2].tolist()

    def _prepare_raw_data(self, input_path, prediction_source):
        raw_data = self._read_input(input_path)
        raw_data = self._merge_prediction_dataframe(raw_data, prediction_source)
        raw_data = raw_data[raw_data["agent_type"].isin(self.ALLOWED_CLASSES)].copy()
        raw_data = raw_data.sort_values(
            by=["scenario_id", "frame_id", "track_id"],
            kind="mergesort",
        )
        return raw_data

    def _build_frame_packets(self, raw_data):
        frame_packets = []
        for (scene_id, frame_id), group in raw_data.groupby(["scenario_id", "frame_id"], sort=False):
            frame_packets.append(self._build_frame_packet_from_group(scene_id, frame_id, group))
        return frame_packets

    def _build_frame_packet_from_group(self, scene_id, frame_id, group):
        group = group.sort_values(by=["track_id"], kind="mergesort")
        ego_rows = group[group["track_id"].astype(str) == "AV"]
        ego_row = ego_rows.iloc[0] if not ego_rows.empty else group.iloc[0]
        obj_rows = group[group["track_id"].astype(str) != "AV"]
        packet = {
            "scene_id": str(scene_id),
            "frame": int(frame_id),
            "frame_id": int(frame_id),
            "time_stamp": self._safe_float(ego_row.get("time_stamp"), float(frame_id)),
            "yaw_rate": self._safe_float(ego_row.get("ego_yawrate"), 0.0),
            "speed": self._safe_float(ego_row.get("ego_v"), 0.0),
            "ego_x": self._safe_float(ego_row.get("ego_x"), 0.0),
            "ego_y": self._safe_float(ego_row.get("ego_y"), 0.0),
            "ego_heading_rad": self._safe_float(ego_row.get("ego_heading_rad"), 0.0),
            "SteerWheelAngle": self._safe_float(ego_row.get("SteerWheelAngle"), np.nan),
            "agents": [],
        }
        for _, row in obj_rows.iterrows():
            dimensions = CommonParams.dimensions_set.get(
                row.get("agent_type", "unknown"),
                CommonParams.dimensions_set["unknown"],
            )
            agent_row = [
                row.get("agent_type", "unknown"),
                self._safe_int(row.get("object_category"), -1),
                self._safe_int(row.get("frame_category"), -1),
                self._safe_float(row.get("rel_x")),
                self._safe_float(row.get("rel_y")),
                dimensions[0],
                dimensions[1],
                self._safe_float(row.get("yaw_rad"), np.nan),
                self._safe_float(row.get("rel_vx")),
                self._safe_float(row.get("rel_vy")),
                str(row.get("track_id")),
            ]
            packet["agents"].append(
                self._attach_prediction_payload(
                    agent_row,
                    row,
                )
            )
        return packet

    def _prediction_dataframe_from_source(self, prediction_source):
        cols = ["scenario_id", "track_id", "frame_id", "agent_class", "hist_traj", "pred_traj", "x", "y", "ptrajs_dict", 
                "object_category", "level", "OX", "OY", "OYAW", "ego_x", "ego_y", "ego_heading_rad", "IS_RESET"]
        prediction_df = pd.DataFrame(prediction_source, columns=cols)
        if prediction_df.empty:
            return prediction_df
        prediction_df = prediction_df.copy()
        prediction_df["scenario_id"] = prediction_df["scenario_id"].astype(str)
        prediction_df["track_id"] = prediction_df["track_id"].astype(str)
        prediction_df["frame_id"] = prediction_df["frame_id"].astype(int)
        prediction_df = prediction_df.sort_values(
            by=["scenario_id", "track_id", "frame_id"],
            kind="mergesort",
        )
        prediction_df["pred_traj"] = prediction_df["pred_traj"].apply(self._normalize_prediction_value)
        prediction_df["hist_traj"] = prediction_df["hist_traj"].apply(self._normalize_prediction_value)
        prediction_df["ptrajs_dict"] = prediction_df["ptrajs_dict"].apply(self._normalize_ptrajs_dict)
        prediction_df[["gt", "pred_ade", "pred_fde", "ptrajs_ade_dict", "ptrajs_fde_dict"]] = self._build_grouped_gt_metrics(prediction_df)
        if "pred_rel_traj" not in prediction_df.columns:
            prediction_df["pred_rel_traj"] = None
        prediction_df["ptrajs_dict"] = prediction_df["ptrajs_dict"].apply(self._dump_json)
        prediction_df["ptrajs_ade_dict"] = prediction_df["ptrajs_ade_dict"].apply(self._dump_json)
        prediction_df["ptrajs_fde_dict"] = prediction_df["ptrajs_fde_dict"].apply(self._dump_json)
        # keep_cols = [
        #     "scenario_id", "track_id", "frame_id", "agent_class", "hist_traj", "pred_traj",
        #     "gt", "pred_ade", "pred_fde", "pred_rel_traj", "ptrajs_dict", "ptrajs_ade_dict", "ptrajs_fde_dict",
        # ]
        keep_cols = [
            "scenario_id", "track_id", "frame_id", "agent_class", "hist_traj",
            "gt", "ptrajs_dict", "ptrajs_ade_dict", "ptrajs_fde_dict", "OX", "OY", "OYAW", "IS_RESET",
        ]
        prediction_df = prediction_df[keep_cols]
        return prediction_df

    def _build_grouped_gt_metrics(self, prediction_df):
        result_df = pd.DataFrame(index=prediction_df.index, columns=["gt", "pred_ade", "pred_fde", "ptrajs_ade_dict", "ptrajs_fde_dict"], dtype=object)
        for _, group in prediction_df.groupby(["scenario_id", "track_id"], sort=False):
            # xy_points = group[["x", "y"]].to_numpy(dtype=float)
            group_indices = group.index.tolist()
            gt_values = []
            ade_values = []
            fde_values = []
            ptrajs_ade_values = []
            ptrajs_fde_values = []
            for idx, _ in enumerate(group_indices):
                prediction_row = group.iloc[idx]
                ego_x = self._safe_float(prediction_row.get("ego_x"))
                ego_y = self._safe_float(prediction_row.get("ego_y"))
                ego_yaw = self._safe_float(prediction_row.get("ego_heading_rad"))
                future_rows = group.iloc[idx + 1:idx + 31]
                crosses_reset = future_rows["IS_RESET"].astype(bool).any()
                gt_traj = future_rows[["x", "y"]].to_numpy(dtype=float).tolist()
                if crosses_reset:
                    gt_traj = []
                    for _, future_row in future_rows.iterrows():
                        raw_x = self._safe_float(future_row.get("OX"))
                        raw_y = self._safe_float(future_row.get("OY"))
                        if not np.isfinite(raw_x + raw_y + ego_x + ego_y + ego_yaw):
                            gt_traj.append([np.nan, np.nan])
                            continue
                        dx = raw_x - ego_x
                        dy = raw_y - ego_y
                        cos_yaw = np.cos(ego_yaw)
                        sin_yaw = np.sin(ego_yaw)
                        gt_traj.append([
                            cos_yaw * dx + sin_yaw * dy,
                            -sin_yaw * dx + cos_yaw * dy,
                        ])
                gt_values.append(gt_traj)
                ade, fde = self._compute_prediction_metrics(group.iloc[idx].get("pred_traj"), gt_traj)
                ade_values.append(ade)
                fde_values.append(fde)
                ptrajs_dict = group.iloc[idx].get("ptrajs_dict") or {}
                ptrajs_ade_dict = {}
                ptrajs_fde_dict = {}
                for sensor_idx, pred_traj in ptrajs_dict.items():
                    sensor_ade, sensor_fde = self._compute_prediction_metrics(pred_traj, gt_traj)
                    ptrajs_ade_dict[str(sensor_idx)] = sensor_ade
                    ptrajs_fde_dict[str(sensor_idx)] = sensor_fde
                ptrajs_ade_values.append(ptrajs_ade_dict)
                ptrajs_fde_values.append(ptrajs_fde_dict)
            result_df.loc[group_indices, "gt"] = gt_values
            result_df.loc[group_indices, "pred_ade"] = ade_values
            result_df.loc[group_indices, "pred_fde"] = fde_values
            result_df.loc[group_indices, "ptrajs_ade_dict"] = ptrajs_ade_values
            result_df.loc[group_indices, "ptrajs_fde_dict"] = ptrajs_fde_values
        return result_df

    def _merge_prediction_dataframe(self, raw_data, prediction_source):
        prediction_df = self._prediction_dataframe_from_source(prediction_source)
        if prediction_df.empty:
            return raw_data
        merged_df = raw_data.copy()
        merged_df["scenario_id"] = merged_df["scenario_id"].astype(str)
        merged_df["track_id"] = merged_df["track_id"].astype(str)
        merged_df["frame_id"] = merged_df["frame_id"].astype(int)
        return merged_df.merge(
            prediction_df,
            how="left",
            on=["scenario_id", "track_id", "frame_id"],
        )

    def _transform_prediction_to_absolute(self, pred_traj):
        if pred_traj is None:
            return None
        pred_array = np.asarray(pred_traj['1'], dtype=float)
        if pred_array.ndim != 2 or pred_array.shape[1] < 2:
            return None
        return pred_array[:, :2].tolist()

    def _convert_prediction_to_relative(self, abs_ptraj):
        return None

    def _compute_prediction_metrics(self, pred_traj, gt_traj):
        if pred_traj is None or gt_traj is None:
            return np.nan, np.nan
        pred_array = np.asarray(pred_traj, dtype=float)
        gt_array = np.asarray(gt_traj, dtype=float)
        if pred_array.ndim != 2 or pred_array.shape[0] == 0:
            return np.nan, np.nan
        if gt_array.ndim != 2 or gt_array.shape[0] == 0:
            return np.nan, np.nan
        pred_eval = pred_array[:len(gt_array), :2]
        distances = np.linalg.norm(pred_eval - gt_array, axis=1)
        ade = float(np.mean(distances)) if len(distances) else np.nan
        fde = float(distances[-1]) if len(distances) else np.nan
        if len(pred_array) != len(gt_array):
            coeff = -1
            ade *= coeff
            fde *= coeff
        return ade, fde

    def _attach_prediction_payload(self, agent_row, row):
        ptrajs_dict = self._normalize_ptrajs_dict(row.get("ptrajs_dict"))
        if not ptrajs_dict:
            return agent_row
        ptrajs_ade_dict = self._normalize_metrics_dict(row.get("ptrajs_ade_dict"))
        ptrajs_fde_dict = self._normalize_metrics_dict(row.get("ptrajs_fde_dict"))
        return agent_row + [ptrajs_dict, ptrajs_ade_dict, ptrajs_fde_dict]

    @staticmethod
    def _should_save_raw_dataframe(save_flag):
        return isinstance(save_flag, str) and save_flag.lower().endswith(".parquet")

    def _resolve_save_dir(self, input_data, save_flag):
        if isinstance(save_flag, str) and save_flag:
            save_dir = os.path.dirname(save_flag)
        elif isinstance(input_data, str):
            save_dir = os.path.dirname(input_data)
        else:
            save_dir = os.getcwd()
        save_dir = save_dir.replace("dataset", "dataset_online")
        os.makedirs(save_dir, exist_ok=True)
        return save_dir

    def _save_frame_packets(self, frame_packets, input_data, save_flag):
        save_dir = self._resolve_save_dir(input_data, save_flag)
        filename = os.path.basename(save_flag) if isinstance(save_flag, str) and save_flag else f"{os.path.splitext(os.path.basename(input_data))[0]}_frame_packets.pkl"
        save_path = os.path.join(save_dir, filename)
        pd.to_pickle(frame_packets, save_path)
        return save_path

    def _save_raw_dataframe(self, raw_data, input_data, save_flag):
        save_dir = self._resolve_save_dir(input_data, save_flag)
        filename = f"{os.path.splitext(os.path.basename(input_data))[0]}_ui.parquet"
        save_path = os.path.join(save_dir, filename)
        raw_data.to_parquet(save_path, index=False)
        return save_path