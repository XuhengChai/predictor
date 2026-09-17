import argparse
import logging
import time

import pandas as pd

from zf_traj_prediction.zf_common import EDatasource
from zf_data_predictor_adapter import Predictor
logger = logging.getLogger(__name__)


def _configure_logging() -> None:
    """Configure the standard project logger for the online lidar simulation."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )


def _read_input(input_path: str) -> pd.DataFrame:
    """Read the lidar input file from CSV or Parquet."""
    if input_path.endswith(".csv"):
        return pd.read_csv(input_path)
    if input_path.endswith(".parquet"):
        return pd.read_parquet(input_path)
    raise ValueError("Unsupported input format. Only .csv and .parquet are supported.")


def _iter_frame_groups(input_df: pd.DataFrame):
    """Yield sorted single-frame record batches for the online lidar simulator."""
    required_columns = [
        "scenario_id", "track_id", "frame_id", "rel_x", "rel_y", "rel_vx", "rel_vy", "agent_type",
        "ego_v", "ego_yawrate", "time_stamp", "SteerWheelAngle", "x", "y", "yaw_rad", "ego_x", "ego_y", "ego_heading_rad",
    ]
    missing_columns = [column for column in required_columns if column not in input_df.columns]
    if missing_columns:
        raise ValueError(f"Missing required input columns: {missing_columns}")
    ordered_df = input_df.copy()
    ordered_df["scenario_id"] = ordered_df["scenario_id"].astype(str)
    ordered_df["track_id"] = ordered_df["track_id"].astype(str)
    ordered_df["frame_id"] = ordered_df["frame_id"].astype(int)
    ordered_df["time_stamp"] = ordered_df["time_stamp"].astype(float)
    if "ego_easting" in ordered_df.columns and "ego_northing" in ordered_df.columns and "ego_orientation" in ordered_df.columns:
        ordered_df["ego_x"] = ordered_df["ego_easting"]
        ordered_df["ego_y"] = ordered_df["ego_northing"]
        ordered_df["ego_heading_rad"] = ordered_df["ego_orientation"]


    ordered_df = ordered_df.sort_values(by=["scenario_id", "frame_id", "time_stamp", "track_id"], kind="mergesort")
    for (scenario_id, frame_id, time_stamp), group in ordered_df.groupby(["scenario_id", "frame_id", "time_stamp"], sort=False):
        yield str(scenario_id), int(frame_id), float(time_stamp), group.to_dict("records")


def main() -> None:
    parser = argparse.ArgumentParser(description="Simulate online lidar prediction from CSV/Parquet input")
    parser.add_argument("--input",
                        # default = r"data\dataset\2026_04_22_155755_lidar_resampled_data_requirement.csv",
                        default = r"data\dataset\2026_04_22_155755_lidar_resampled.parquet",
                        help="Input CSV or Parquet file path")
    parser.add_argument("--output",
                        default= r"data\dataset\2026_04_22_155755_lidar_output.csv",
                        help="Output CSV file path")
    parser.add_argument("--ego-pose-mode", default="gps", choices=["gps", "integration"], help="Ego pose source for local coordinates")
    parser.add_argument("--realtime", action="store_true", help="Replay frames using timestamp deltas")
    args = parser.parse_args()
    _configure_logging()
    if not args.output.lower().endswith(".csv"):
        raise ValueError("Output must be a .csv file because the simulator writes append-friendly flattened rows")

    input_df = _read_input(args.input)
    # input_df.to_csv("demo.csv", index=False)  # Initialize output CSV with header
    ego_pose_mode = EDatasource.ONLINE_GPS if args.ego_pose_mode == "gps" else EDatasource.ONLINE_NO_GPS
    predictor = Predictor(csv_path=args.input, data_source=ego_pose_mode)
    previous_scene_id = None
    previous_time_stamp = None

    try:
        for scenario_id, frame_id, time_stamp, frame_rows in _iter_frame_groups(input_df):
            if args.realtime and previous_scene_id == scenario_id and previous_time_stamp is not None:
                time.sleep(max(0.0, time_stamp - previous_time_stamp))
            if previous_scene_id != scenario_id:
                # Clear recorder history on scenario switches to avoid cross-scene history contamination.
                predictor.feature_pps.data_recorder.clear()
            previous_scene_id = scenario_id
            previous_time_stamp = time_stamp

            try:
                frame_preds = predictor.predict_frame(frame_rows, save_path=args.output, ego_pose_mode=ego_pose_mode)
                logger.info(
                    "Processed lidar frame: scenario_id=%s frame_id=%s time_stamp=%s objects=%s predictions=%s",
                    scenario_id,
                    frame_id,
                    time_stamp,
                    len(frame_rows),
                    len(frame_preds),
                )
            except Exception:
                logger.exception(
                    "Failed lidar frame: scenario_id=%s frame_id=%s time_stamp=%s",
                    scenario_id,
                    frame_id,
                    time_stamp,
                )
                continue
    finally:
        if predictor.online_csv_writer is not None:
            predictor.online_csv_writer.close()


if __name__ == "__main__":
    main()
