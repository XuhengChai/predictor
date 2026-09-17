## metric_sind cal_metric.py
import numpy as np
import pickle

from .zf_feature_preprocess import FeaturePreprocess
from .predictor.zf_predictor_kalman import CAPredictor, CVPredictor
from .predictor.zf_predictor_tnt import TNTPredictor
from .predictor.zf_predictor_vru_class import VRUClassPredictor
from .predictor.zf_predictor_imm_mdn import IMMMDNPredictor
# from evaluator.trajectory_evaluator import evaluation
from .zf_common import EAgentProp, EAgentType, EDatasource, CommonParams, EMacro
from .input_process.zf_fix_his_len import HistoryFixConfig, HistoryLengthFixer
from .ui_frame_packet_exporter import UIFramePacketExporter, OnlineLidarCsvWriter
import pandas as pd
import os
from tqdm import tqdm
import time
import logging
import json
import argparse

logger = logging.getLogger(__name__)
class Predictor:
    def __init__(
        self,
        csv_path="",
        data_source=EDatasource.OFFLINE_GPS,
        tnt_infer_impl: str = "standard",
        output_suffix: str = "",
        history_fix_config = None,
        output_dir: str = "",
    ):
        self.csv_file_path = csv_path
        self.data_source = data_source
        self.tnt_infer_impl = tnt_infer_impl
        self.scene_id = ""
        self.output_dir = output_dir
        self.max_history_length = CommonParams.obj_history * CommonParams.obj_frequency  # e.g., 3s * 10Hz = 30
        def build_tnt_predictor():
            return TNTPredictor(tnt_infer_impl=tnt_infer_impl)

        self.class_methods = {
            EAgentType.PEDESTRIAN: CVPredictor(),
            EAgentType.BICYCLE: CVPredictor(),
            # EAgentType.BICYCLE: CAPredictor(),
            EAgentType.MOTORCYCLIST: CVPredictor(),
            # EAgentType.MOTORCYCLIST: CVPredictor(),
            EAgentType.DUMMY_CLS_CA: CAPredictor(),
            EAgentType.DUMMY_CLS_CV: CVPredictor(),
            EAgentType.DUMMY_CLS_TNT: build_tnt_predictor(),
            EAgentType.DUMMY_CLS_IMM_MDN: IMMMDNPredictor(),
        }
        self.vru_classifier = VRUClassPredictor()
        self.feature_pps = FeaturePreprocess(data_source=data_source)
        self.history_fixer = HistoryLengthFixer(self.max_history_length, CommonParams.obj_frequency)
        self.history_fix_config = history_fix_config or HistoryLengthFixer.keep_last_seconds(1.0)
        self.predictions = []  # will be list of rows: [scene_id, track_id, frame_id, agent_class, hist_traj, mode_prob, pred_traj]
        self.predictions_per_file = []  # will be list of rows: [scene_id, track_id, frame_id, agent_class, hist_traj, mode_prob, pred_traj]
        self.ui_frame_packet_exporter = UIFramePacketExporter()
        self.online_csv_writer = None
        self.online_output_path = ""
        # output pickle path (pandas-compatible)
        self.output_suffix = f"_{output_suffix}" if output_suffix else ""
        self.output_pkl_path = self._build_prediction_output_path(csv_path, ".pkl")
        self.output_csv_path = self._build_prediction_output_path(csv_path, ".csv")

    def predict_frame(self, input_data, save_path: str, actual_agent_type="cyclist", 
                      ego_pose_mode: EDatasource = EDatasource.ONLINE_NO_GPS, 
                      method_list = None,
                      change_origin=True):
        # input_data = [
        #     {
        #         "scenario_id": "2026_04_22_155755", "track_id": "1", "frame_id": 20,
        #         "rel_x": 0.858862, "rel_y": -3.49246, "rel_vx": 0.017123, "rel_vy": -0.14148, "rel_psi":-0.2,
        #         "agent_type": "cyclist", "time_stamp": 1776866276.7,"obj_score" : 11.68,
        #         "ego_v": 0.304635, "ego_yawrate": -0.04323,"SteerWheelAngle": 0.1,
        #         "ego_easting": 523010, "ego_northing": 4140000, "ego_orientation": 0.785398
        #     },
        #     {
        #         "scenario_id": "2026_04_22_155755", "track_id": "id_2", "frame_id": 20,
        #         "rel_x": 0.858862, "rel_y": -3.49246, "rel_vx": 0.017123, "rel_vy": -0.14148, "rel_psi":-0.2,
        #         "agent_type": "vehicle", "time_stamp": 1776866276.7,"obj_score" : 13.9,
        #         "ego_v": 0.304635, "ego_yawrate": -0.04323, "SteerWheelAngle": 0.1,
        #         "ego_easting": 523010, "ego_northing": 4140000, "ego_orientation": 0.785398
        #     },
        # ]
        """Run one online lidar frame through the existing predictor flow and flush the output row set."""
        if self.online_csv_writer is None or self.online_output_path != save_path:
            if self.online_csv_writer is not None:
                self.online_csv_writer.close()
            self.online_csv_writer = OnlineLidarCsvWriter(save_path)
            self.online_output_path = save_path
        frame_dict = self.feature_pps.get_frame_dict(input_data, change_origin)
        if not frame_dict:
            return []
        frame_id = next(iter(frame_dict.keys()))
        # print(f"Processing frame {frame_id} with {len(frame_dict[frame_id][EMacro.OBJ_DATA])} objects.")
        frame_data = frame_dict[frame_id]
        if self.scene_id and self.scene_id != frame_data[EMacro.SCENE_ID]:
            # Reset per-scene history to avoid mixing trajectories across scenes.
            self.feature_pps.data_recorder.clear()
        self.scene_id = frame_data[EMacro.SCENE_ID]
        self.data_source = ego_pose_mode
        frame_meta = frame_data.get(EMacro.ORIGIN_META)
        agents = self.feature_pps.update(
            frame_data[EMacro.EGO_DATA],
            frame_data[EMacro.OBJ_DATA],
            ego_pose_mode,
            frame_meta=frame_meta,
        )
        frame_preds = []
        if method_list is not None:
            frame_preds = self.predict_agents(agents, method_list=method_list, origin_meta=frame_meta) if len(agents) else []
        # if frame_preds:
        #     self.predictions.extend(frame_preds)
        #     self.predictions_per_file.extend(frame_preds)
        self.online_csv_writer.write_frame(input_data, agents, frame_preds)
        return frame_preds

    def _prediction_file_name(self, extension: str) -> str:
        return f"predictions{self.output_suffix}{extension}"

    def _build_prediction_output_path(self, csv_path: str, extension: str) -> str:
        if self.output_dir:
            return os.path.join(self.output_dir, self._prediction_file_name(extension))
        if csv_path:
            return csv_path.replace(".csv", f"_pred{self.output_suffix}{extension}")
        return self._prediction_file_name(extension)

    def save_ui_frame_packets(self, input_path: str = None, save_hmi_pkl=True):
        source_path = input_path or self.csv_file_path
        if not source_path:
            raise ValueError("input_path or csv_path is required to export UI frame packets")
        return self.ui_frame_packet_exporter.export(
            source_path,
            prediction_source=self.predictions_per_file,
            save_flag=save_hmi_pkl,
        )

    @staticmethod
    def _safe_int(value, default=-1):
        try:
            if pd.isna(value):
                return default
        except TypeError:
            pass
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _safe_float(value, default=np.nan):
        try:
            if pd.isna(value):
                return default
        except TypeError:
            pass
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _compute_ui_prediction_metrics(self, ptraj, gt_traj):
        if ptraj is None or gt_traj is None or len(gt_traj) == 0:
            return np.nan, np.nan
        pred_array = np.asarray(ptraj, dtype=float)
        gt_array = np.asarray(gt_traj, dtype=float)
        if pred_array.ndim != 2 or pred_array.shape[0] == 0 or gt_array.ndim != 2 or gt_array.shape[0] == 0:
            return np.nan, np.nan
        pred_eval = pred_array[:gt_array.shape[0], :2]
        gt_eval = gt_array[:pred_eval.shape[0], :2]
        distances = np.linalg.norm(pred_eval - gt_eval, axis=1)
        if not len(distances):
            return np.nan, np.nan
        coeff = 1
        if len(ptraj) != len(gt_traj):
            coeff = -1
        return coeff*float(np.mean(distances)), coeff*float(distances[-1])

    def predict_agents_ui(self, request, enabled_sensor_indices=None):
        if request.ndim != 2 or request.shape[1] != len(EAgentProp):
            return {}
        if enabled_sensor_indices is None:
            enabled_sensor_indices = [0, 1, 2, 3]
        enabled_sensor_indices = {
            int(sensor_idx)
            for sensor_idx in enabled_sensor_indices
            if self._safe_int(sensor_idx, -1) >= 0
        }
        prediction_methods = [
            (EAgentType.DUMMY_CLS_CA, 0),
            (EAgentType.DUMMY_CLS_CV, 1),
            (EAgentType.DUMMY_CLS_TNT, 2),
            (EAgentType.DUMMY_CLS_IMM_MDN, 3),
        ]
        prediction_methods = [
            method for method in prediction_methods
            if method[1] in enabled_sensor_indices
        ]
        if not prediction_methods:
            return {}
        agents_60 = request.astype(object, copy=False)

        block_size = self.max_history_length + CommonParams.obj_predict * CommonParams.obj_frequency
        if agents_60.shape[0] < block_size:
            return {}
        valid_row_count = (agents_60.shape[0] // block_size) * block_size
        if valid_row_count == 0:
            return {}
        agents = agents_60[:valid_row_count].copy()
        agents = self.calculate_all_agents_props(agents, compute_heading=False)
        levels = np.asarray([self._safe_float(value, -1) for value in agents[:, EAgentProp.LEVEL]], dtype=float)
        timestamps = np.asarray([self._safe_float(value, -1) for value in agents[:, EAgentProp.TIME_STAMP]], dtype=float)
        predict_level_filter = agents[(levels > 2) & (timestamps != -1)]
        if len(predict_level_filter) == 0:
            return {}
        classes = np.asarray([self._safe_int(value, EAgentType.PEDESTRIAN) for value in predict_level_filter[:, EAgentProp.CLASS]], dtype=int)
        predict_class_filter = predict_level_filter[classes < 3]
        if len(predict_class_filter) == 0:
            return {}
        
        track_ids = agents[:, EAgentProp.TRACK_ID]
        predict_track_ids = predict_class_filter[:, EAgentProp.TRACK_ID]
        unique_track_ids = np.unique(predict_track_ids)
        results = {}
        for track_id in unique_track_ids.tolist():
            agent_rows = agents[track_ids == track_id]
            surroundings_rows = agents[track_ids != track_id]
            agent = agent_rows[:self.max_history_length]
            if agent[0, EAgentProp.TIME_STAMP] == -1:
                continue
            gt_rows = agent_rows[self.max_history_length:]
            if len(gt_rows):
                valid_gt_mask = np.asarray([
                    self._safe_float(value, -1) != -1
                    for value in gt_rows[:, EAgentProp.TIME_STAMP]
                ], dtype=bool)
                gt_rows = gt_rows[valid_gt_mask]
            first_frame_id = self._safe_int(agent[0, EAgentProp.FRAME_ID], -1)
            valid_surroundings_mask = (
                    surroundings_rows[:, EAgentProp.FRAME_ID]
                    <= first_frame_id + self.max_history_length)
            surroundings = surroundings_rows[valid_surroundings_mask]

            gt_traj = None
            if len(gt_rows):
                gt_traj = np.asarray(gt_rows[:, [EAgentProp.X, EAgentProp.Y]], dtype=float)
            method_predictions = {}
            for dummy_cls, sensor_idx in prediction_methods:
                prediction_agent = agent
                if dummy_cls in (EAgentType.DUMMY_CLS_TNT, EAgentType.DUMMY_CLS_IMM_MDN):
                    predictor = self.class_methods[EAgentType.DUMMY_CLS_CV]
                    prediction_agent = predictor.filter_agent(agent.copy())
                ptraj = self.predict_agent(
                    prediction_agent,
                    surroundings,
                    dummy_cls=dummy_cls,
                )
                pred_traj = None if ptraj is None else ptraj.get("1")
                if pred_traj is None:
                    continue
                ade = np.nan
                fde = np.nan
                if gt_traj is not None and len(gt_traj):
                    ade, fde = self._compute_ui_prediction_metrics(pred_traj, gt_traj)
                method_predictions[str(sensor_idx)] = {
                    "pred_traj": pred_traj,
                    "ade": ade,
                    "fde": fde,
                }
            if method_predictions:
                results[str(track_id)] = method_predictions
        return results

    def get_track_ids_by_distance(self, agents, av_track_id="AV"):
        track_ids = agents[:, EAgentProp.TRACK_ID]
        unique_ids, first_idx, counts = np.unique(
            track_ids,
            return_index=True,
            return_counts=True
        )
        last_rows = agents[first_idx + counts - 1]
        av_mask = unique_ids == av_track_id
        av_row = last_rows[av_mask][0]
        distances = np.hypot(
            last_rows[:, EAgentProp.X].astype(np.float64)- av_row[EAgentProp.X].astype(np.float64),
            last_rows[:, EAgentProp.Y].astype(np.float64) - av_row[EAgentProp.Y].astype(np.float64)
        )
        valid_mask = ~av_mask
        distances[av_mask] = np.inf
        nearest_idx = np.argmin(distances)
        return unique_ids[nearest_idx], distances[nearest_idx]
        # sorted_idx = np.argsort(distances[valid_mask])
        # return (
        #     unique_ids[valid_mask][sorted_idx],
        #     distances[valid_mask][sorted_idx]
        # )

    # filter agents to predict
    def predict_agents(self, agents, method_list = None, origin_meta=None):
        """Predict for all agents in a frame and return a list of prediction records.

        Each record is a dict: {scene_id, track_id, frame_id, ptraj}
        """
        # IF VX VY conf == filter
        agents = self.calculate_all_agents_props(agents)
        # ilevel = agents[:, EAgentProp.OBJECT_CATEGORY]
        # predict_level_filter = agents[ilevel.astype(float) > 2]
        # class_threshold = EAgentType.VEHICLE.value
        # icls = predict_level_filter[:, EAgentProp.CLASS]
        # predict_class_filter = predict_level_filter[icls < class_threshold]

        # predict_track_ids = predict_class_filter[:, EAgentProp.TRACK_ID]
        # unique_val, indices = np.unique(predict_track_ids, return_index=True)
        nearest_id, distance = self.get_track_ids_by_distance(agents)
        track_ids = agents[:, EAgentProp.TRACK_ID]
        frame_preds = []
        # for id in unique_val.tolist():
        for id in [nearest_id]:
            agent = agents[track_ids == id]
            surroundings = agents[track_ids != id]
            if CommonParams.DEBUG:
                df_agent = pd.DataFrame(agent, columns=EAgentProp.names())
                df_surround = pd.DataFrame(surroundings, columns=EAgentProp.names())
            '''
            Risk 1: change the pedestrian to bicycle, and change the cyclist to pedestrian
            '''
            cls = EAgentType(int(agent[-1, EAgentProp.CLASS]))
            if cls != EAgentType.PEDESTRIAN:
                agent[:, EAgentProp.CLASS] = EAgentType.BICYCLE.value
            else:
                agent[:, EAgentProp.CLASS] = EAgentType.PEDESTRIAN.value
            # elif cls == EAgentType.CYCLIST:
            #     agent[:, EAgentProp.CLASS] = EAgentType.PEDESTRIAN.value
            # else:
            #     print(f"ERROR: Unknown type")
            '''
            Risk 2: change all class to pedestrian/bicycle
            '''
            # agent[:, EAgentProp.CLASS] = EAgentType.BICYCLE.value
            # agent[:, EAgentProp.CLASS] = EAgentType.PEDESTRIAN.value

            # cls_type, conf = self.vru_classifier.predict_class(agent, use_model=True)
            # agent[:, EAgentProp.CLASS] = cls_type
            # check if it is LOW_Displacement
            # his_xy = agent[:, [EAgentProp.X, EAgentProp.Y]].astype(float)
            # diff_gt = his_xy[-10:, :] - his_xy[-11:-1, :]  # history 1s
            # tmp = np.sqrt(np.sum(diff_gt ** 2, axis=1))  # [batch_size, len]
            # total_dis = float(np.sum(tmp, axis=0, keepdims=True))
            # if total_dis < 0.5:
            #     continue
            # start = time.perf_counter()
            track_id = agent[0, EAgentProp.TRACK_ID]
            frame_id = agent[-1, EAgentProp.FRAME_ID]
            total_history_error = 0
            # print(f"Predicting for scene {self.scene_id}, track {track_id}, frame {frame_id}")
            if len(agent) != self.max_history_length:
                # print(f"Warning: {self.scene_id}, agent {track_id} {frame_id} has history length {len(agent)}, "
                #       f"expected {self.max_history_length}. Skipping.")
                continue
            '''
            Risk 3: Insufficient historical trajectory data
            '''
            # fixed_agent, history_meta = self.history_fixer.apply(agent=agent, config=self.history_fix_config)
            # agent = self.calculate_single_agent_props(fixed_agent)
            # total_history_error = history_meta['total_history_error']
            # if CommonParams.DEBUG and history_meta["mode"] != "none":
            #     print(
            #         f"History fix applied: scene={self.scene_id}, track={track_id}, frame={frame_id}, "
            #         f"mode={history_meta['mode']}, fill={history_meta['fill_method']}, "
            #         f"confidence={history_meta['confidence']:.3f}, "
            #         f"observed={history_meta['observed_frames']}/{history_meta['expected_frames']}, "
            #         f"total_history_error={history_meta['total_history_error']:.3f}"
            #     )
            # prd = self.class_methods[EAgentType.DUMMY_CLS_CV]
            # agent_filter = prd.filter_agent(agent.copy())
            agent_filter = agent
            # ptraj = self.predict_agent(agent_filter, surroundings)
            # only predict DUMMY_CLS_CV CA TNT IMM_MDN
            ptrajs_dict = {}
            if method_list is None:
                method_list = [
                    EAgentType.DUMMY_CLS_CA,
                    EAgentType.DUMMY_CLS_CV,
                    EAgentType.DUMMY_CLS_TNT,
                    # EAgentType.DUMMY_CLS_IMM_MDN
                ]
            for dummy_cls in method_list:
                if dummy_cls == EAgentType.DUMMY_CLS_TNT \
                    or dummy_cls == EAgentType.DUMMY_CLS_IMM_MDN:
                    prd = self.class_methods[EAgentType.DUMMY_CLS_CV]
                    agent_filter = prd.filter_agent(agent.copy())
                ptraj = self.predict_agent(agent_filter, surroundings, dummy_cls=dummy_cls)
                ptrajs_dict[dummy_cls.value-10] = ptraj
            ptraj = ptrajs_dict[method_list[0].value-10]
            # end = time.perf_counter()
            # print(f"执行时间: {end - start:.6f} 秒")
            if not ptrajs_dict:
                print(f"Warning: {self.scene_id}, agent {track_id} {frame_id} predict None!!!")
            # take a representative track id for this agent (first row)
            # convert numeric class to lowercase name (e.g., 'bicycle')
            if agent.shape[0] > 0:
                try:
                    agent_class_name = EAgentType(int(agent[0, EAgentProp.CLASS])).name.lower()
                except Exception:
                    # fallback to the numeric value as string
                    agent_class_name = str(int(agent[0, EAgentProp.CLASS]))
            else:
                agent_class_name = None
            hist_traj = agent_filter[:, [EAgentProp.X, EAgentProp.Y]].astype(float)
            row = [
                self.scene_id,
                track_id,
                int(frame_id),
                agent_class_name,
                hist_traj,
                ptraj,
                agent[-1, EAgentProp.X],
                agent[-1, EAgentProp.Y],
                ptrajs_dict,
                agent[-1, EAgentProp.OBJECT_CATEGORY],
                agent[-1, EAgentProp.LEVEL],
                agent[-1, EAgentProp.OX],
                agent[-1, EAgentProp.OY],
                agent[-1, EAgentProp.OYAW],
                origin_meta.get("origin_x", np.nan) if origin_meta else np.nan,
                origin_meta.get("origin_y", np.nan) if origin_meta else np.nan,
                origin_meta.get("origin_theta", np.nan) if origin_meta else np.nan,
                bool(origin_meta.get("origin_reset", False)) if origin_meta else False,
            ]
            frame_preds.append(row)
        return frame_preds

    def predict_agent(self, agent, surroundings, dummy_cls: EAgentType = None):
        cls = np.unique(agent[:, EAgentProp.CLASS])[0]
        if cls not in self.class_methods:
            cls = EAgentType.PEDESTRIAN
        if dummy_cls is not None:
            cls = dummy_cls
        prd = self.class_methods[cls]
        ptraj = prd.predict_traj(agent, surroundings)
        return ptraj

    def predict_agent_multi(self, agent, surroundings, dummy_cls_list=[]):
        if not len(dummy_cls_list):
            return None
        ptraj_list = {}
        for dummy_cls in dummy_cls_list:
            prd = self.class_methods[dummy_cls]
            ptraj = prd.predict_traj(agent, surroundings)
            ptraj_list[dummy_cls] = ptraj
        return ptraj_list

    def loop_load(self, save_ui_pkl=False):
        frames_dict = self.feature_pps.get_frames_dict(self.csv_file_path)
        for frame_id, data in frames_dict.items():
            # if frame_id > 79:
            #     continue
            agents = self.feature_pps.update(data[EMacro.EGO_DATA], data[EMacro.OBJ_DATA], self.data_source)
            self.scene_id = data[EMacro.SCENE_ID]
            if len(agents):
                frame_preds = self.predict_agents(agents)
                if frame_preds:
                    self.predictions.extend(frame_preds)
        # after processing all frames, save predictions to a pickle
        self.save_predictions()
        if save_ui_pkl:
            self.save_ui_frame_packets(self.csv_file_path, save_hmi_pkl=True)

    def loop_load_dataset(self, dataset_path, posfix=".parquet", save_ui_pkl=True):
        output_dir = self.output_dir or os.path.dirname(dataset_path)
        self.output_csv_path = os.path.join(output_dir, self._prediction_file_name(".csv"))
        self.output_pkl_path = os.path.join(output_dir, self._prediction_file_name(".pkl"))
        file_list = [f for f in os.listdir(dataset_path) if f.endswith(posfix)]
        for file_name in tqdm(file_list):
            self.predictions_per_file.clear()
            file_path = os.path.join(dataset_path, file_name)
            frames_dict = self.feature_pps.get_frames_dict(file_path)
            for frame_id, data in frames_dict.items():
                # if frame_id > 79:
                #     continue
                agents = self.feature_pps.update(data[EMacro.EGO_DATA], data[EMacro.OBJ_DATA], self.data_source)
                self.scene_id = data[EMacro.SCENE_ID]
                if len(agents):
                    frame_preds = self.predict_agents(agents, data.get(EMacro.ORIGIN_META))
                    if frame_preds:
                        self.predictions.extend(frame_preds)
                        self.predictions_per_file.extend(frame_preds)
            if save_ui_pkl:
                self.save_ui_frame_packets(file_path, save_hmi_pkl=True)
        self.save_predictions()

    def save_predictions(self, path=None):
        """Save the accumulated predictions list to a pickle file and a CSV file."""
        out_path = path if path is not None else self.output_pkl_path
        output_dir = os.path.dirname(out_path)
        csv_output_dir = os.path.dirname(self.output_csv_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        if csv_output_dir:
            os.makedirs(csv_output_dir, exist_ok=True)

        # Deduplicate by (scene_id, track_id, frame_id)
        # Deduplicate rows by (scene_id, track_id, frame_id, mode_prob)
        seen = set()
        # deduped = []
        # for rec in self.predictions:
        #     key = (rec.get("scene_id"), rec.get("track_id"), rec.get("frame_id"))
        #     if key in seen:
        #         continue
        #     seen.add(key)
        #     deduped.append(rec)
        deduped = self.predictions
        # Convert to DataFrame and save via pandas (pickle + csv)
        try:
            with open(out_path, "wb") as f:
                pickle.dump(deduped, f)
            cols = ["scene_id", "track_id", "frame_id", "agent_class", "hist_traj", "pred_traj", "x", "y", "ptrajs_dict","object_category","level", "OX", "OY", "OYAW", "ego_x", "ego_y", "ego_heading_rad", "IS_RESET"]
            df = pd.DataFrame(deduped, columns=cols)
            # df["hist_traj"] = df["hist_traj"].apply(lambda value: json.dumps(value, default=_json_default, ensure_ascii=False, separators=(",", ":")))
            # df["pred_traj"] = df["pred_traj"].apply(lambda value: json.dumps(value, default=_json_default, ensure_ascii=False, separators=(",", ":")) if value is not None else "")
            # df["ptrajs_dict"] = df["ptrajs_dict"].apply(lambda value: json.dumps(value, default=_json_default, ensure_ascii=False, separators=(",", ":")) if value is not None else "")
            df.to_csv(self.output_csv_path, index=False)
            if CommonParams.DEBUG:
                print(f"Saved {len(deduped)} predictions (from {len(self.predictions)}) to {out_path}")
        except Exception as e:
            print(f"Failed to save predictions to {out_path}: {e}")

    def calculate_single_agent_props(self, points, dt=0.1, compute_heading=True):
        n = len(points)
        if n < 2:
            return points
        # 确保按时间戳排序
        # if points.shape[1] > EAgentProp.TIME_STAMP:
        #     sorted_idx = np.argsort(points[:, EAgentProp.TIME_STAMP])
        #     points = points[sorted_idx]
        # 提取位置
        x = points[:, EAgentProp.X].astype(float)
        y = points[:, EAgentProp.Y].astype(float)
        # 计算时间步长
        if dt is None and points.shape[1] > EAgentProp.TIME_STAMP:
            timestamps = points[:, EAgentProp.TIME_STAMP]
            dt_array = np.diff(timestamps)
            dt_array = np.concatenate([dt_array, [dt_array[-1]]]) if len(dt_array) > 0 else np.ones(n) * 0.1
        else:
            dt_array = np.full(n, dt)
        # 1. 计算速度
        dx = np.diff(x)  # Δx
        dy = np.diff(y)  # Δy
        vx = dx / dt_array[:-1]
        vy = dy / dt_array[:-1]
        vx = np.concatenate([[vx[0]], vx])
        vy = np.concatenate([[vy[0]], vy])
        # vx = np.concatenate([vx, [vx[-1]]])
        # vy = np.concatenate([vy, [vy[-1]]])
        v = np.sqrt(vx ** 2 + vy ** 2)
        # 将结果存储回数组
        points[:, EAgentProp.VX] = vx
        points[:, EAgentProp.VY] = vy
        points[:, EAgentProp.V] = v
        if compute_heading:
            yaw = np.arctan2(vy, vx)
            yaw = np.unwrap(yaw, discont=np.pi)
            yaw_rate = np.zeros_like(yaw)
            yaw_diff = np.diff(yaw)
            yaw_rate[1:] = yaw_diff / dt_array[1:]
            points[:, EAgentProp.YAW] = yaw
            points[:, EAgentProp.YR] = yaw_rate
        return points

    def calculate_all_agents_props(self, agent_data, dt=0.1, compute_heading=True):
        result = agent_data.copy()
        ids = agent_data[:, EAgentProp.TRACK_ID]
        unique_track_ids = np.unique(ids)
        for track_id in unique_track_ids:
            mask = (ids == track_id)
            agent_points = agent_data[mask]
            calculated = self.calculate_single_agent_props(agent_points, dt, compute_heading=compute_heading)
            result[mask] = calculated
        return result


def build_history_fix_runs():
    return [
        # ("keep_last_1s", HistoryLengthFixer.keep_last_seconds(1.0)),
        # ("keep_last_2s", HistoryLengthFixer.keep_last_seconds(2.0)),
        # ("drop_middle_10_linear", HistoryLengthFixer.drop_middle_frames(10, fill_method="linear")),
        # ("drop_middle_10_bezier", HistoryLengthFixer.drop_middle_frames(10, fill_method="bezier")),
        # ("drop_middle_20_linear", HistoryLengthFixer.drop_middle_frames(20, fill_method="linear")),
        # ("drop_middle_20_bezier", HistoryLengthFixer.drop_middle_frames(20, fill_method="bezier")),
        ("drop_random_15_linear_seed0", HistoryLengthFixer.drop_random_frames(15, fill_method="linear", random_seed=0)),
        ("drop_random_15_bezier_seed0", HistoryLengthFixer.drop_random_frames(15, fill_method="bezier", random_seed=0)),
    ]


def run_prediction_once(args, history_fix_config: HistoryFixConfig, run_output_dir = None) -> None:
    offline_data = Predictor(
        csv_path=args.csv_path,
        tnt_infer_impl=args.tnt_infer_impl,
        output_suffix=args.output_suffix,
        history_fix_config=history_fix_config,
        output_dir=run_output_dir or "",
    )
    start_time = time.perf_counter()
    if args.csv_path:
        offline_data.loop_load()
    else:
        offline_data.loop_load_dataset(args.dataset_path, posfix=args.posfix)
    elapsed = time.perf_counter() - start_time
    print(f"TNT infer impl: {offline_data.tnt_infer_impl}")
    print(f"Prediction pickle: {offline_data.output_pkl_path}")
    print(f"Prediction csv: {offline_data.output_csv_path}")
    print(f"Elapsed time: {elapsed:.3f} s")
    evaluation(argv=[], file_path=offline_data.output_pkl_path, output_dir=run_output_dir)


def run_history_fix_experiments(args) -> None:
    for run_name, history_fix_config in build_history_fix_runs():
        run_output_dir = os.path.join(args.history_fix_output_root, run_name)
        os.makedirs(run_output_dir, exist_ok=True)
        print(f"\n=== Running history fix config: {run_name} ===")
        run_prediction_once(args, history_fix_config=history_fix_config, run_output_dir=run_output_dir)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run trajectory prediction with selectable TNT inference implementation")
    parser.add_argument("--csv-path", default="", help="Single CSV file path to predict")
    parser.add_argument("--dataset-path", default="./data/dataset", help="Dataset directory for batch prediction")
    parser.add_argument("--posfix", default=".parquet", help="Dataset file suffix, e.g. .parquet or .csv")
    parser.add_argument(
        "--tnt-infer-impl",
        default="standard",
        choices=["standard", "opt_nms"],
        help="Choose which TNT inference backend to use",
    )
    parser.add_argument(
        "--output-suffix",
        default="standard",
        help="Optional suffix appended to prediction output files for result comparison",
    )
    parser.add_argument(
        "--run-history-fix-experiments",
        action="store_true",
        help="Run all temporary history-fix configurations sequentially and save each run into its own folder",
    )
    parser.add_argument(
        "--history-fix-output-root",
        default="./data/risks/history_fix_runs",
        help="Root directory for sequential history-fix experiment outputs",
    )
    args = parser.parse_args()
    args.run_history_fix_experiments = False
    if args.run_history_fix_experiments:
        run_history_fix_experiments(args)
    else:
        run_prediction_once(args, history_fix_config=HistoryLengthFixer.keep_last_seconds(1.0))
