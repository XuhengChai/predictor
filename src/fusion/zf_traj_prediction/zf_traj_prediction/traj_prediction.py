# -*- coding: utf-8 -*-

import rclpy
import os
import csv
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from interface.msg import FusedDataFrame
from .zf_common import EDatasource, EAgentType
from .zf_data_predictor_adapter import Predictor

class TrajPrediction(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.data_source = EDatasource.ONLINE_NO_GPS
        self._home_dir = os.path.expanduser("~")
        self._bool_need_predict = True
        self._predict = Predictor(data_source=self.data_source)
        self.subscription = self.create_subscription(
            FusedDataFrame,
            "/fused_info",
            self.fused_callback,
            qos_profile,
        )

    def fused_callback(self, msg):
        # msg to dict_like:
                # input_data = [
        #     {
        #         "scenario_id": "2026_04_22_155755", "track_id": "1", "frame_id": 20,
        #         "rel_x": 0.858862, "rel_y": -3.49246, "rel_vx": 0.017123, "rel_vy": -0.14148, "rel_psi":-0.2,
        #         "agent_type": "cyclist", "time_stamp": 1776866276.7,"obj_score" : 11.68,
        #         "ego_v": 0.304635, "ego_yawrate": -0.04323,"SteerWheelAngle": 0.1,
        #     },
        #     {
        #         "scenario_id": "2026_04_22_155755", "track_id": "id_2", "frame_id": 20,
        #         "rel_x": 0.858862, "rel_y": -3.49246, "rel_vx": 0.017123, "rel_vy": -0.14148, "rel_psi":-0.2,
        #         "agent_type": "vehicle", "time_stamp": 1776866276.7,"obj_score" : 13.9,
        #         "ego_v": 0.304635, "ego_yawrate": -0.04323, "SteerWheelAngle": 0.1,
        #     },
        # ]
        input_data = []
        for agent in msg.agents:
            input_data.append({
                "scenario_id": msg.scenario_id,
                "track_id": agent.track_id,
                "frame_id": msg.frame_id,
                "rel_x": agent.rel_x,
                "rel_y": agent.rel_y,
                "rel_vx": agent.rel_vx,
                "rel_vy": agent.rel_vy,
                "rel_psi": agent.rel_psi,
                "agent_type": agent.agent_type,
                "time_stamp": msg.time_stamp,
                "obj_score": agent.obj_score,
                "ego_v": msg.ego_v,
                "ego_yawrate": msg.ego_yawrate,
                "SteerWheelAngle": msg.steer_wheel_angle,
            })
        output_path = os.path.join(msg.save_path, msg.scenario_id.replace("-", "_") + ".csv")
        # output_path = os.path.join(self._home_dir, msg.save_path, msg.scenario_id.replace("-", "_") + ".csv")
        method_list = None
        if self._bool_need_predict:
            method_list = [EAgentType.DUMMY_CLS_TNT, EAgentType.DUMMY_CLS_CA]
        self._bool_need_predict = not self._bool_need_predict
        self._predict.predict_frame(input_data, 
                    output_path, 
                    ego_pose_mode=self.data_source,
                    method_list = method_list)
        # print(
        #     "scenario={}, frame={}, ego_v={}, agents={}".format(
        #         msg.scenario_id,
        #         msg.frame_id,
        #         msg.ego_v,
        #         len(msg.agents),
        #     )
        # )
        # with open(output_path, "w") as f:
        #     writer = csv.writer(f)
        #     agent = input_data[0]
        #     writer.writerow([
        #             agent["scenario_id"], agent["track_id"], agent["frame_id"], agent["rel_x"], agent["rel_y"],
        #             agent["rel_vx"], agent["rel_vy"], agent["rel_psi"], agent["agent_type"], agent["time_stamp"],
        #             agent["obj_score"], agent["ego_v"], agent["ego_yawrate"], agent["SteerWheelAngle"]
        #         ])
        #     f.flush()

        # for agent in msg.agents:
        #     print(
        #         "track_id={}, type={}, "
        #         "position=({}, {}), velocity=({}, {})".format(
        #             agent.track_id,
        #             agent.agent_type,
        #             agent.rel_x,
        #             agent.rel_y,
        #             agent.rel_vx,
        #             agent.rel_vy,
        #         )
        #     )


def main(args=None):
    rclpy.init(args=args)
    node = TrajPrediction("traj_prediction")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()