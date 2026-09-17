# -*- coding: utf-8 -*-
"""
Created on Fri Jan  5 12:39:33 2024

@author: ZHANG Jing, Yiming Chen
"""
# ros2 launch zf_hal_sync sync_tracking_can_launch.py
import rclpy
import pandas as pd
import numpy as np
import threading
import pickle
import copy
from rclpy.node import Node
from enum import Enum, IntEnum
from collections import deque
# from std_msgs.msg import String, Float32MultiArray
from can_msgs.msg import CanDatas
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interface.msg import (MultiArrayWithHeader, VehicleInfo,
                           NodeState, HmiAgent,
                           PredictAgent,FusedDataFrame)


from fusion.cam_preprocess import CamPrePrc
from fusion.cam_postprocess import CamPostProc

#colcon build --packages-select fusion
class States(Enum):

    INITIALIZATION = 10
    ENABLED = 30
    DOWNGRADE = 50
    ERROR = 60

class ESensorSource(IntEnum):
    CAM = 0
    CAMPP = CAM + 1

# %% Fusion online
class Fusion(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameter("replay_rosbag", True)
        self.declare_parameter("show_time", False)
        self.declare_parameter("show_main_time", False)
        self.declare_parameter("record_result", False)
        self.declare_parameter("show_timer_FPS", False)
        self.declare_parameter("show_data", False)
        self.replay_rosbag = self.get_parameter("replay_rosbag").value
        self.show_time = self.get_parameter("show_time").value
        self.record_result = self.get_parameter("record_result").value
        self.show_timer_FPS = self.get_parameter("show_timer_FPS").value
        self.show_data = self.get_parameter("show_data").value
        self.show_main_time = self.get_parameter("show_main_time").value

        self._calc_times = deque(maxlen=10)
        # self.data_fused = pd.DataFrame()

        self.cam_preprc = CamPrePrc()
        self.cam_postprc = CamPostProc()
        
       
        self.msg_cam = MultiArrayWithHeader()
        self.cam_bboxes = []
        self.cam_update = False
        self.lock_cam_info = threading.Lock()
        self.lock_ego_info = threading.Lock()
        self.msg_ego = CanDatas()

        self.d_cam = {}
        self.t_cam = 0.
        self.t_cam_record = "0"
        self.veh_spd = 0.
        self.ego_yawrate = 0.
        self.steer_angle = 0.
        self.artic_angle = 0.
        self.scenario_id = "2026"
        self.frame_id = 0

        # self.pub_cols_str = ["id", "class", "motion", "source", "status"]
        self.pub_cols_str = ["id", "class", "motion", "source", "status"] #zhijuan
        self.fused_cols_str = ["fusion_id", "class", "motion", "source", "fusion_status"]
        self.pub_cols_float = [
                                "pos_x",
                                "pos_y",
                                "width",
                                "length",
                                "heading",
                                # "lifetime", #zhijuan
                                "class_confidence",
                                # "rel_spd", #zhijuan
                                "rel_spd_lat",
                                "rel_spd_lon",
                              ]
        self.cam_objs = []
        self.cam_objs_pp = []
        self.predict_agents = []
        self.camera_timestamp = 0.0
        self.cam_objs_agents = HmiAgent()
        self.cam_objs_pp_agents = HmiAgent()

        self.extrap_time_thrd = 0.03
        self.delay_time_thrd = 0.2
        self.last_timer_cb_time = self.get_clock().now()
        self.msg_node_state = NodeState()
        self.node_state = States.INITIALIZATION.value
        self.qos_profile = QoSProfile(
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1)
        cb_group1 = MutuallyExclusiveCallbackGroup()
        cb_group2 = MutuallyExclusiveCallbackGroup()

        self._sub_cam_sts = self.create_subscription(NodeState,
                                                     "/perception_state",
                                                     self._cb_cam_state,
                                                     self.qos_profile,
                                                     callback_group=cb_group1)

        self._subscription_cam = self.create_subscription(
                                        MultiArrayWithHeader,
                                        "/tracked_objects",
                                        self._callback_cam_preproc,
                                        self.qos_profile,
                                        callback_group=cb_group1)
        self._subscription_info = self.create_subscription(
                                        VehicleInfo, "/vehicle_info",
                                        self._callback_vehicle_info,
                                        self.qos_profile,
                                        callback_group=cb_group1)
        self._timer = self.create_timer(0.1, self._fusion_state_callback,
                                        callback_group=cb_group1)
        self._timer_fusion = self.create_timer(0.1, self._fusion_callback,
                                               callback_group=cb_group2)
        self.pub_fuse_res = self.create_publisher(FusedDataFrame,
                                                  "/fused_info",
                                                  self.qos_profile)
        self.pub_state = self.create_publisher(NodeState, "/fusion_state",
                                               self.qos_profile)
        topic_list = ["/cam/hal/sensor_objs", "/campp/hal/sensor_objs"]
        self.pub_single_sensor = []
        for topic_name in topic_list:
            self.pub_single_sensor.append(self.create_publisher(HmiAgent,
                                          topic_name, self.qos_profile))
        self.record_data = {}
        self._received_ego_data = True
        self._received_camera_data = False

        

    def _cb_cam_state(self, msg):
        if msg.node_state == States.ERROR.value:
            self.node_state = States.DOWNGRADE.value

    def _fusion_callback(self):
        start_time = self.get_clock().now()
        if self.show_timer_FPS:
            # 1.Calculate FPS
            elapsed_timer_cb_time = (
                start_time - self.last_timer_cb_time).nanoseconds / 1e9
            self.last_timer_cb_time = self.get_clock().now()
            fusion_fps = 1 / elapsed_timer_cb_time
            self.get_logger().info(
                "Fusion FPS: {:.2f} elapsed step time {:.4f}."
                .format(fusion_fps, elapsed_timer_cb_time))
        self.node_state = States.ENABLED.value
        #======================================================================================
        # with self.lock_cam_info:
            
        # cam_pp
        
        self.pub_single_sensor[ESensorSource.CAM].publish(self.cam_objs_agents)
        self.pub_single_sensor[ESensorSource.CAMPP].publish(self.cam_objs_pp_agents)

        #=======================================================================================

        
        t1 = self.get_clock().now()
        self._data_synch()
        t2 = self.get_clock().now()
        self._fusion()
        t3 = self.get_clock().now()
        if self._received_ego_data and self._received_camera_data:
            self._publish_fused_data()
            self.frame_id += 1
        # self._publish_fused_result()
  

    def _data_synch(self):
        start_time = self.get_clock().now()
        self.cur_time = self.get_clock().now().nanoseconds * 1e-9
        return

    def _data_extrap(self, data, t_diff):
        if data is None:
            pass
        elif len(data):
            for k in data:
                data[k]["pos_x"] = t_diff * data[k]["rel_spd_lon"] + data[k]["pos_x"]
                data[k]["pos_y"] = t_diff * data[k]["rel_spd_lat"] + data[k]["pos_y"]

    def _callback_cam_preproc(self, msg):
        start_time = self.get_clock().now()
        str_list = [str(msg.header.stamp.sec), ".", str(msg.header.stamp.nanosec).zfill(9)]
        self.t_cam_record = "".join(str_list)
        self.t_cam = float(self.t_cam_record)
        self.msg_cam = msg
        tracks = np.array(msg.array.data, dtype=np.float32)
        array_shape = [dim.size for dim in msg.array.layout.dim]
        if len(tracks) == 0:
            self.cam_bboxes = []
            return
        tracks = np.reshape(tracks, array_shape)
        self.cam_bboxes = tracks.tolist()
        # print("self.cam_bboxes ------", self.cam_bboxes)
        start_time_cam_pp1 = self.get_clock().now()
        _d_cam = {}
        self.cam_objs = []
        _d_cam = self.cam_preprc.process_boxes(self.cam_bboxes)
        for key, value in _d_cam.items():
            prop_list = [str(value["class"]),
                        "10",
                        "10",
                        str(value["pos_x"]),
                        str(value["pos_y"]),
                        str(value["width"]),
                        str(value["length"]),
                        str(value["heading"]),
                        "0",
                        "0",
                        str(key)]
            str_prop = " ".join(prop_list)
            self.cam_objs.append(str_prop)
        if self.show_time:
            timecost = (self.get_clock().now()-start_time_cam_pp1).nanoseconds / 1e9
            self.get_logger().info("CAM_PP1 callback timecost: {} sec".format(timecost))
        start_time_cam_pp2 = self.get_clock().now()
        self.cam_objs_pp = []
        self.cam_postprc.update_input(_d_cam,
                                self.t_cam,
                                self.veh_spd,
                                self.steer_angle)
        self.cam_postprc.main()
        # TODO: change form to dict in camera preprocess
        self.d_cam = {k: v.copy() for k, v in self.cam_postprc.d_pp.items()} #copy.deepcopy(self.cam_pp.d_pp)
        for key, value in self.cam_postprc.d_pp.items():
            self.d_cam[key] = value.copy()
            prop_list = [str(value["class"]),
                        "11",
                        "11",
                        str(value["pos_x"]),
                        str(value["pos_y"]),
                        str(value["width"]),
                        str(value["length"]),
                        str(value["heading"]),
                        str(value["rel_spd_lon"]),
                        str(value["rel_spd_lat"]),
                        str(key) # track_id
                        ]
            str_prop = " ".join(prop_list)
            self.cam_objs_pp.append(str_prop)
        
        if self.show_time:
            timecost = (self.get_clock().now()-start_time_cam_pp2).nanoseconds / 1e9
            self.get_logger().info("CAM_PP2 callback timecost: {} sec".format(timecost))
        with self.lock_cam_info:
            self.cam_objs_agents.agent = [row[:] for row in self.cam_objs]
            self.cam_objs_pp_agents.agent = [row[:] for row in self.cam_objs_pp]
            self.predict_agents = [self._to_predict_agent(track_id, data)
                                    for track_id, data in self.cam_postprc.d_pp.items()
                                ]
            self.camera_timestamp = msg.time_stamp
            self._received_camera_data = True
        if self.show_main_time:
            timecost = (self.get_clock().now()-start_time).nanoseconds / 1e9
            self.get_logger().info("CAM pp1 callback timecost: {} sec"
                                   .format(timecost))
            start_time = self.get_clock().now()

    def _agent_type_from_class(self, object_class):
        class_text = str(object_class).strip().lower()
        class_map = {
            # 数字类别需要按照项目定义调整
            "0": "vehicle",
            "1": "pedestrian",
            "2": "cyclist",
            "3": "vehicle",

            "car": "vehicle",
            "vehicle": "vehicle",
            "truck": "vehicle",
            "bus": "vehicle",
            "van": "vehicle",
            "person": "pedestrian",
            "pedestrian": "pedestrian",
            "bicycle": "cyclist",
            "cyclist": "cyclist",
            "motorcycle": "cyclist",
            "motorbike": "cyclist",
        }
        return class_map.get(class_text, "unknown")

    def _to_predict_agent(self, track_id, data):
        agent_msg = PredictAgent()
        agent_msg.track_id = str(track_id)
        agent_msg.rel_x = data.get("pos_x")
        agent_msg.rel_y = data.get("pos_y")
        agent_msg.rel_vx = data.get("rel_spd_lon")
        agent_msg.rel_vy = data.get("rel_spd_lat")
        agent_msg.rel_psi = data.get("heading")
        object_class = data.get("class", "unknown")
        agent_msg.agent_type = self._agent_type_from_class(object_class)
        agent_msg.object_category = -1
        agent_msg.frame_category = -1
        agent_msg.obj_score = data.get("class_confidence", float("nan"))
        return agent_msg

    def _callback_vehicle_info(self, msg):
        # TODO: if no vehicle info received???
        with self.lock_ego_info:
            self.veh_spd = msg.v_ego
            self.steer_angle = msg.steering_angle
            self.ego_yawrate = float(msg.tractor_yawrate) # rad/s
            self._received_ego_data = True
        self.artic_angle = msg.artic_angle

    def _fusion_state_callback(self):
        # Publish node state
        self.msg_node_state.header.stamp = self.get_clock().now().to_msg()
        self.msg_node_state.node_state = self.node_state
        self.msg_node_state.watchdog_signal = (
            1 - self.msg_node_state.watchdog_signal)
        self.pub_state.publish(self.msg_node_state)

    def _fusion(self):
        start_time = self.get_clock().now()
        

    def _calc_fusion_state(self, st_srr):
        # print ("***st_srr", st_srr, "st_srr***")
        srr_operation_st = [
            "Initializing", "Fully Operational", "Performance Limited"
        ]
        if st_srr not in srr_operation_st:
            self.node_state = States.DOWNGRADE.value
            self.get_logger().warning("srr operation state is {} !"
                                      .format(st_srr))

    def _save_fused_info(self):
        with open("fused_info.txt", "a") as file:
            file.write(str(self.cur_time) + "_"
                       + str(self.get_clock().now()) + "\n")

    # #zhijuan,public dict
    # def _publish_fused_result(self):
    #     start_time = self.get_clock().now()
    #     # print("publishing...")
    #     self.fuse_msg = FusedDataFrame()
    #     # self.fuse_msg.data_ipm = self.msg_ipm
    #     self.fuse_msg.veh_info = self.msg_ego
    #     self.fuse_msg.header.stamp = self.get_clock().now().to_msg()
    #     self.fuse_msg.col_str = self.pub_cols_str #self.pub_cols_str = ["id", "class", "motion", "source", "status"]
    #     self.fuse_msg.col_float = self.pub_cols_float #self.pub_cols_float = ["pos_x","pos_y",

    #     #=======================================================
    #     #zhijuan
    #     # print("###################################################################")
    #     # print("self.matched.fused_result:")
    #     # print(self.matched.fused_result)
    #     self.fuse_msg.data_str = []
    #     self.fuse_msg.data_float = []
    #     self.fuse_msg.shape_str = []
    #     self.fuse_msg.shape_float = []
    #     # fuse_pub_time=self.get_clock().now().nanoseconds / 1e9
    #     str_list = [str(self.fuse_msg.header.stamp.sec), ".", str(self.fuse_msg.header.stamp.nanosec).zfill(9)]
    #     fuse_pub_time = "".join(str_list)    
    #     time_str_list=list()

    #     #sensor_time_dict={"srr":self.t_srr,"cv":self.t_cam,"ipm1":self.t_ipm} 
    #     #sensor_tt_time_dict={"srr":c,"cv":self.tt_cam,"ipm1":self.tt_ipm}
    #     #sensor_last_time_dict={"srr":self.t_srr_last,"cv":self.t_cam_last,"ipm1":self.t_ipm_last}
    #     #for key in sensor_time_dict.keys():
    #         #if sensor_time_dict[key]>sensor_last_time_dict[key]:
    #             #time_str_list.append(str(sensor_time_dict[key]))
    #         #else:
    #             #if sensor_tt_time_dict[key] is None:
    #                 #time_str_list.append("")
    #             #else:
    #                 #time_str_list.append(str(sensor_tt_time_dict[key]))
    #     #     if key in self.matched.fused_result["source"]:
    #     #         time_str_list.append(str(sensor_time_dict[key]))
    #     #     else:
    #     #         time_str_list.append('nan')
    #     #time_str_list.append(str(fuse_pub_time))
    #     time_str_list=[str(self.t_cam_record), str(fuse_pub_time)]
    #     self.fuse_msg.time_str=time_str_list # time_str, srr,cv,cam,fuse
    #     self.pub_fuse_res.publish(self.fuse_msg)


    #public df
    def _publish_fused_data(self):
        start_time = self.get_clock().now()
        # print("publishing...")
        self.fuse_msg = FusedDataFrame()
        self.fuse_msg.header.stamp = start_time.to_msg()
        # print("=-----------------", self.fuse_msg.get_fields_and_field_types())
        # self.fuse_msg.scenario_id = self.scenario_id
        self.fuse_msg.save_path = self.msg_cam.save_path
        self.fuse_msg.scenario_id = self.msg_cam.save_path.split("/")[-1]
        self.fuse_msg.frame_id = self.frame_id
        with self.lock_ego_info:
            self.fuse_msg.ego_v = float(self.veh_spd)
            self.fuse_msg.ego_yawrate = float(self.ego_yawrate)
            self.fuse_msg.steer_wheel_angle = float(self.steer_angle)
        with self.lock_cam_info:
            self.fuse_msg.agents = self.predict_agents
            # self.fuse_msg.time_stamp = self.camera_timestamp
            self.fuse_msg.time_stamp = start_time.nanoseconds * 1.0 / 1e9

        self.pub_fuse_res.publish(self.fuse_msg)
        if self.show_data:
            self.get_logger().info(
                "Published FusedDataFrame: "
                "scenario={}, frame={}, agents={}".format(
                    self.fuse_msg.scenario_id,
                    self.fuse_msg.frame_id,
                    len(self.fuse_msg.agents)
                )
            )

        # # self.fuse_msg.data_ipm = self.msg_ipm
        # self.fuse_msg.veh_info = self.msg_ego
        # self.fuse_msg.header.stamp = self.get_clock().now().to_msg()
        # self.fuse_msg.col_str = self.pub_cols_str 
        # self.fuse_msg.col_float = self.pub_cols_float 
        # self.fuse_msg.data_str = []
        # self.fuse_msg.data_float = []
        # self.fuse_msg.shape_str = []
        # self.fuse_msg.shape_float = []
        # self.pub_fuse_res.publish(self.fuse_msg)


def main(args=None):
    rclpy.init(args=args)
    fusion_node = Fusion("fusion_node")
    executor = MultiThreadedExecutor()
    executor.add_node(fusion_node)
    executor.spin()
    # rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# %%
