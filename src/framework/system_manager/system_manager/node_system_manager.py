# !/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2023 ZF TDIP
# All rights reserved.
# Authors: Yiming Chen <yiming.chen@zf.com>

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interface.msg import VehicleInfo, NodeState, SystemState

from jtop import jtop
from enum import Enum
from collections import deque
from system_manager.node_ego_fusion import EgoFusionNode


class States(Enum):

    INITIALIZATION = 10
    READY = 20
    READY_WITH_DOWNGRADE = 25
    ENABLED = 30
    ENABLED_WITH_DOWNGRADE = 35
    ACTIVE = 40
    ACTIVE_WITH_DOWNGRADE = 45
    ERROR = 60


class Flags(Enum):

    OFF = 10
    ON = 20


class SystemManagerNode(Node):

    """
    Publish system state accroding to software, hardware, watchdog states
    Software state is calculated by received node states
    Hardware state is calculated by monitored device status
    Watchdog state is calculated by received watchdog signals
    Publish trigger state accroding to right turn signal
    ----------------------------------------------------------------------
    BSD State:
    Initialization         = 10
    Enable                 = 30
    Downgraded enable      = 35
    Active                 = 40
    Downgraded active      = 45
    Error                  = 60
    Warning State from Planning Node:
    Off = 10
    On  = 20
    ----------------------------------------------------------------------
    ICA State:
    Initialization         = 10
    Ready                  = 20
    Downgraded ready       = 25
    Enable                 = 30
    Downgraded enable      = 35
    Active                 = 40
    Downgraded active      = 45
    Error                  = 60
    Trigger State:
    Off = 10
    On  = 20
    Active Flage from Control Node:
    Off = 10
    On  = 20
    ----------------------------------------------------------------------
    Hardware State:
    Initialization = 10, Enabled = 30, Downgrade = 50, Error = 60
    Software State:
    Initialization = 10, Enabled = 30, Downgrade = 50, Error = 60
    Node State:
    Initialization = 10, Enabled = 30, Downgrade = 50, Error = 60
    ----------------------------------------------------------------------
    Watchdog State:
    Initialization = 10, Enabled = 30, Downgrade = 50, Error = 60
    """

    def __init__(self, name):

        super().__init__(name)
        self.get_logger().info("System manager node initialization start...")
        qos_profile_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        # qos_profile_reliable = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RELIABLE,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1)
        self.declare_parameter('use_image_raw', True)
        self.declare_parameter('show_log_in_terminal', True)
        self.declare_parameter('save_log_in_file', True)
        self.use_image_raw = self.get_parameter('use_image_raw').value
        self.show_log_in_terminal = self.get_parameter(
                                               'show_log_in_terminal').value
        self.save_log_in_file = self.get_parameter('save_log_in_file').value

        callback_group1 = MutuallyExclusiveCallbackGroup()
        callback_group2 = MutuallyExclusiveCallbackGroup()
        callback_group3 = MutuallyExclusiveCallbackGroup()

        # Timers
        self.timer = self.create_timer(
            0.2, self.timer_callback, callback_group=callback_group3)
        # Subscriptions
        self.sub_vehicle_state = self.create_subscription(
            VehicleInfo, "/vehicle_info", self.vehicle_info_callback,
            qos_profile_best_effort, callback_group=callback_group1)

        self.sub_perception_state = self.create_subscription(
            NodeState, "/perception_state", self.perception_state_callback,
            qos_profile_best_effort, callback_group=callback_group2)

        self.sub_fusion_state = self.create_subscription(
            NodeState, "/fusion_state", self.fusion_state_callback,
            qos_profile_best_effort, callback_group=callback_group2)

        self.sub_planning_bsd_state = self.create_subscription(
            NodeState, "/planning_bsd_state", self.planning_bsd_state_callback,
            qos_profile_best_effort, callback_group=callback_group2)

        self.sub_planning_ica_state = self.create_subscription(
            NodeState, "/planning_ica_state", self.planning_ica_state_callback,
            qos_profile_best_effort, callback_group=callback_group2)

        self.sub_control_state = self.create_subscription(
            NodeState, "/control_state", self.control_state_callback,
            qos_profile_best_effort, callback_group=callback_group2)

        self.sub_ego_state = self.create_subscription(
            NodeState, "/ego_state", self.ego_state_callback,
            qos_profile_best_effort, callback_group=callback_group2)

        self.sub_can_state = self.create_subscription(
            NodeState, "/can_state", self.can_state_callback,
            qos_profile_best_effort, callback_group=callback_group2)
        # Publishers
        self.pub_system_state = self.create_publisher(
            SystemState, "/system_state", qos_profile_best_effort)
        # System state initialization
        self.system_state_msg = SystemState()
        self.system_bsd_state = 10
        self.system_ica_state = 10
        self.system_bsd_state_hmi = 0
        self.system_ica_state_hmi = 0
        # Hardware monitor initialization
        self.jetson = jtop(interval=0.5)
        self.jetson.start()
        self.cpu_state = 0
        self.gpu_state = 0
        self.memory_state = 0
        self.disk_state = 0
        self.hardware_state = 10
        # Software monitor initialization
        self.sw_bsd_state = 10
        self.sw_ica_state = 10
        # Monitor following 7 node states:
        #     Detection, Fusion, Planning_BSD, Planning_ICA,
        #     Control, Ego State, CAN
        self.node_name_list = [
            "Detection", "Fusion", "Planning_BSD", "Planning_ICA",
            "Control", "Ego State", "CAN"]
        self.state_count = len(self.node_name_list)
        self.node_state_list = [10]*self.state_count
        self.state_stamp_list = [0]*self.state_count
        # Watchdog monitor initialization
        # 3 consecutive watchdog wrong signals cause watchdog state error
        self.tolerance_len = 3
        self.tolerance_lists = [[0]*self.tolerance_len
                                for _ in range(self.state_count)]
        # [Detection watchdog state, Fusion watchdog state,
        #  Planning_BSD watchdog state, Planning_ICA watchdog state,
        #  Control watchdog state, Ego State watchdog state,
        #  CAN watchdog state]
        self.watchdog_state_list = [10]*self.state_count
        self.watchdog_state = 10
        self.watchdog_signal_list = [0]*self.state_count
        self.watchdog_signal_list_last = [1]*self.state_count
        # Vehicle state initialization
        self.vehicle_state_msg = VehicleInfo()
        self.acc_pedal_hist = deque(maxlen=5)
        self.acc_pedal_hist.append(0)
        self.cnt_hyst_spd = 0
        self.vehicle_spd_ready_last = False
        self.turn_signal_last = 2
        self.rising_edge_count = 0
        self.rising_edge_time_last = None
        self.falling_edge_time = None
        self.trigger_time = None
        self.trigger_state = 10
        self.get_logger().info("System manager node initialization finished.")

    def timer_callback(self):

        self.hardware_state = self.hardware_monitor(self.jetson.stats)
        self.sw_bsd_state, self.sw_ica_state = self.software_monitor()
        self.watchdog_state = self.watchdog_monitor()
        self.trigger_state_monitor(self.vehicle_state_msg)
        self.system_bsd_state, self.system_ica_state = self.system_monitor()
        self.system_state_hmi()
        self.system_state_msg.bsd_state = self.system_bsd_state
        self.system_state_msg.ica_state = self.system_ica_state
        self.system_state_msg.trigger_state = self.trigger_state
        self.system_state_msg.bsd_state_hmi = self.system_bsd_state_hmi
        self.system_state_msg.ica_state_hmi = self.system_ica_state_hmi
        self.system_state_msg.cpu_state = self.cpu_state
        self.system_state_msg.gpu_state = self.gpu_state
        self.system_state_msg.memory_state = self.memory_state
        self.system_state_msg.disk_state = self.disk_state
        self.system_state_msg.watchdog_state_list = self.watchdog_state_list
        self.system_state_msg.node_state_list = self.node_state_list
        self.system_state_msg.trigger_state_hmi = self.trigger_state
        self.system_state_msg.sw_bsd_state = self.sw_bsd_state
        self.system_state_msg.sw_ica_state = self.sw_ica_state
        self.system_state_msg.watchdog_signal = (
                             1 - self.system_state_msg.watchdog_signal)
        self.system_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_system_state.publish(self.system_state_msg)

        self.get_logger().info("#"*10+"System State"+"#"*10)
        self.get_logger().info(
            "BSD state: {}".format(self.system_bsd_state))
        self.get_logger().info(
            "ICA state: {}".format(self.system_ica_state))
        self.get_logger().info(
            "Hardware state: {}".format(self.hardware_state))
        self.get_logger().info(
            "Softrware state: {}".format(self.node_state_list))
        self.get_logger().info(
            "Watchdog state: {}".format(self.watchdog_state))
        self.get_logger().info(
            "Trigger State: {}".format(self.trigger_state))
        self.get_logger().info("#"*32)

    def system_monitor(self):

        sys_states = [self.hardware_state, self.watchdog_state]
        if all(value_ == 30 for value_ in sys_states):
            sys_bsd_state = self.sw_bsd_state
            sys_ica_state = self.sw_ica_state
        elif 10 in sys_states:
            sys_bsd_state = 10
            sys_ica_state = 10
        elif 60 in sys_states:
            sys_bsd_state = 60
            sys_ica_state = 60
        elif 50 in sys_states:
            if self.sw_bsd_state in [30, 40]:
                sys_bsd_state = self.sw_bsd_state + 5
            else:
                sys_bsd_state = self.sw_bsd_state
            if self.sw_ica_state in [20, 30, 40]:
                sys_ica_state = self.sw_ica_state + 5
            else:
                sys_ica_state = self.sw_ica_state
        else:
            sys_bsd_state = 60
            sys_ica_state = 60
            self.get_logger().error(
                "Unexpected software states without Planning state: {}"
                .format(sys_states))
        return sys_bsd_state, sys_ica_state

    def system_state_hmi(self):
        # Node HMI requires BSD and ICA system state:
        # uint8 Error     = 2
        # uint8 Downgrade = 1
        # uint8 Other     = 0
        if self.system_bsd_state in [10, 30, 40]:
            self.system_bsd_state_hmi = 0
        elif self.system_bsd_state in [35, 45]:
            self.system_bsd_state_hmi = 1
        else:
            self.system_bsd_state_hmi = 2

        if self.system_ica_state in [10, 20, 30, 40]:
            self.system_ica_state_hmi = 0
        elif self.system_ica_state in [25, 35, 45]:
            self.system_ica_state_hmi = 1
        else:
            self.system_ica_state_hmi = 2

    def hardware_monitor(self, jetson_state):

        self.cpu_state = self.cpu_monitor(jetson_state)
        self.gpu_state = self.gpu_monitor(jetson_state)
        self.memory_state = self.memory_monitor(jetson_state)
        self.disk_state = self.disk_monitor(self.jetson)
        state_list = [
            self.cpu_state, self.gpu_state, self.memory_state, self.disk_state]

        if all(value_ == 30 for value_ in state_list) and len(state_list) == 4:
            return 30
        elif 10 in state_list and len(state_list) == 4:
            self.get_logger().info("Hardware states: {}".format(state_list))
            return 10
        elif 60 in state_list and len(state_list) == 4:
            self.get_logger().error("Hardware states: {}".format(state_list))
            return 60
        elif 50 in state_list and len(state_list) == 4:
            self.get_logger().warning("Hardware states: {}".format(state_list))
            return 50
        else:
            self.get_logger().error(
                "Unexpected hardware states: {}".format(state_list))
            return 60

    def cpu_monitor(self, jetson_state):
        cpu_percent_ = round(
            (jetson_state["CPU1"] + jetson_state["CPU2"]
             + jetson_state["CPU3"] + jetson_state["CPU4"]
             + jetson_state["CPU5"] + jetson_state["CPU6"]
             + jetson_state["CPU7"] + jetson_state["CPU8"])/8, 1)
        cpu_temperature_ = round(jetson_state["Temp CPU"], 1)
        if self.show_log_in_terminal:
            self.get_logger().info(
                "CPU load: {}%, CPU temperature: {}C"
                .format(cpu_percent_, cpu_temperature_))
        if (cpu_percent_ >= 0.0 and cpu_percent_ < 100.0
           and cpu_temperature_ >= 0.0 and cpu_temperature_ < 90.0):
            return 30
        elif (cpu_percent_ == 100.0
              or (cpu_temperature_ >= 90.0 and cpu_temperature_ < 100.0)):
            return 50
        else:
            return 60

    def gpu_monitor(self, jetson_state):
        gpu_percent_ = round(jetson_state["GPU"], 1)
        gpu_temperature_ = round(jetson_state["Temp GPU"], 1)
        if self.show_log_in_terminal:
            self.get_logger().info(
                "GPU load: {}%, GPU temperature: {}C"
                .format(gpu_percent_, gpu_temperature_))
        if (gpu_percent_ >= 0.0 and gpu_percent_ < 100.0
           and gpu_temperature_ >= 0.0 and gpu_temperature_ < 90.0):
            return 30
        elif (gpu_percent_ == 100.0
              or (gpu_temperature_ >= 90.0 and gpu_temperature_ < 100.0)):
            return 50
        else:
            return 60

    def memory_monitor(self, jetson_state):
        memory_usage_ = round(jetson_state["RAM"] * 100, 1)
        if self.show_log_in_terminal:
            self.get_logger().info("RAM load: {}%".format(memory_usage_))
        if memory_usage_ >= 0.0 and memory_usage_ < 96.0:
            return 30
        elif memory_usage_ >= 96.0 and memory_usage_ < 100.0:
            return 50
        else:
            return 60

    def disk_monitor(self, jetson):
        disk_usage_ = round(
            (jetson.disk["used"]/jetson.disk["total"]) * 100, 1)
        if self.show_log_in_terminal:
            self.get_logger().info("Disk usage: {}%".format(disk_usage_))
        if disk_usage_ >= 0.0 and disk_usage_ < 96.0:
            return 30
        elif disk_usage_ >= 96.0 and disk_usage_ < 100.0:
            return 50
        else:
            return 60

    def perception_state_callback(self, data):
        index_ = 0
        self.node_state_list[index_] = data.node_state
        self.watchdog_signal_list[index_] = data.watchdog_signal
        stamp_ = round(
            data.header.stamp.sec + data.header.stamp.nanosec * 1e-9, 4)
        self.state_stamp_list[index_] = stamp_
        self.watchdog_signal_monitor(index_)
        self.watchdog_signal_list_last[index_] = data.watchdog_signal

    def fusion_state_callback(self, data):
        index_ = 1
        self.node_state_list[index_] = data.node_state
        self.watchdog_signal_list[index_] = data.watchdog_signal
        stamp_ = round(
            data.header.stamp.sec + data.header.stamp.nanosec * 1e-9, 4)
        self.state_stamp_list[index_] = stamp_
        self.watchdog_signal_monitor(index_)

    def planning_bsd_state_callback(self, data):
        index_ = 2
        self.node_state_list[index_] = data.node_state
        self.watchdog_signal_list[index_] = data.watchdog_signal
        stamp_ = round(
            data.header.stamp.sec + data.header.stamp.nanosec * 1e-9, 4)
        self.state_stamp_list[index_] = stamp_
        self.watchdog_signal_monitor(index_)

    def planning_ica_state_callback(self, data):
        index_ = 3
        self.node_state_list[index_] = data.node_state
        self.watchdog_signal_list[index_] = data.watchdog_signal
        stamp_ = round(
            data.header.stamp.sec + data.header.stamp.nanosec * 1e-9, 4)
        self.state_stamp_list[index_] = stamp_
        self.watchdog_signal_monitor(index_)

    def control_state_callback(self, data):
        index_ = 4
        self.node_state_list[index_] = data.node_state
        self.watchdog_signal_list[index_] = data.watchdog_signal
        stamp_ = round(
            data.header.stamp.sec + data.header.stamp.nanosec * 1e-9, 4)
        self.state_stamp_list[index_] = stamp_
        self.watchdog_signal_monitor(index_)

    def ego_state_callback(self, data):
        index_ = 5
        self.node_state_list[index_] = data.node_state
        self.watchdog_signal_list[index_] = data.watchdog_signal
        stamp_ = round(
            data.header.stamp.sec + data.header.stamp.nanosec * 1e-9, 4)
        self.state_stamp_list[index_] = stamp_
        self.watchdog_signal_monitor(index_)

    def can_state_callback(self, data):
        index_ = 6
        self.node_state_list[index_] = data.node_state
        self.watchdog_signal_list[index_] = data.watchdog_signal
        stamp_ = round(
            data.header.stamp.sec + data.header.stamp.nanosec * 1e-9, 4)
        self.state_stamp_list[index_] = stamp_
        self.watchdog_signal_monitor(index_)

    def vehicle_info_callback(self, data):
        self.vehicle_state_msg = data

    def software_monitor(self):
        sw_states = self.node_state_list[:2]+self.node_state_list[4:]
        bsd_state = self.node_state_list[2]
        ica_state = self.node_state_list[3]
        if all(value_ == 30 for value_ in sw_states):
            if bsd_state in [10, 30, 35, 40, 45, 60]:
                sw_bsd_state = bsd_state
            else:
                sw_bsd_state = 60
                self.get_logger().error(
                 "Unexpected Planning BSD state: {}"
                 .format(bsd_state))
            if ica_state in [10, 20, 25, 30, 35, 40, 45, 60]:
                sw_ica_state = ica_state
            else:
                sw_ica_state = 60
                self.get_logger().error(
                 "Unexpected Planning ICA state: {}"
                 .format(ica_state))
        elif 10 in sw_states:
            sw_bsd_state = 10
            sw_ica_state = 10
        elif 60 in sw_states:
            sw_bsd_state = 60
            sw_ica_state = 60
        elif 50 in sw_states:
            if bsd_state in [30, 40]:
                sw_bsd_state = bsd_state + 5
            elif bsd_state in [10, 35, 45, 60]:
                sw_bsd_state = bsd_state
            else:
                sw_bsd_state = 60
                self.get_logger().error(
                 "Unexpected Planning BSD state: {}"
                 .format(bsd_state))
            if ica_state in [20, 30, 40]:
                sw_ica_state = ica_state + 5
            elif ica_state in [10, 25, 35, 45, 60]:
                sw_ica_state = ica_state
            else:
                sw_ica_state = 60
                self.get_logger().error(
                 "Unexpected Planning ICA state: {}"
                 .format(ica_state))
        else:
            sw_bsd_state = 60
            sw_ica_state = 60
            self.get_logger().error(
                "Unexpected software states without Planning state: {}"
                .format(sw_states))
        return sw_bsd_state, sw_ica_state

    def watchdog_monitor(self):
        current_time_ = round(self.get_clock().now().nanoseconds*1e-9, 4)
        for index_, value_ in enumerate(self.state_stamp_list):
            # missing watchdog signal for 1 sec
            if current_time_ - value_ < 1 and current_time_ - value_ >= 0:
                self.watchdog_state_list[index_] = 30
            else:
                self.watchdog_state_list[index_] = 60
                self.get_logger().error(
                    "Watchdog signal from node {} is time out!"
                    .format(self.node_name_list[index_]))
                self.get_logger().error(
                    "Watchdog signal stamp list: {}"
                    .format(self.state_stamp_list))

        if (self.watchdog_state_list.count(30) == self.state_count
           and len(self.watchdog_state_list) == self.state_count):
            return 30
        elif (self.watchdog_state_list.count(10) > 0
              and len(self.watchdog_state_list) == self.state_count):
            self.get_logger().info(
                "Watchdog states: {}".format(self.watchdog_state_list))
            return 10
        else:
            self.get_logger().error(
                "Watchdog states: {}".format(self.watchdog_state_list))
            return 60

    def watchdog_signal_monitor(self, index):

        if ((self.watchdog_signal_list[index] in [0, 1])
                and (self.watchdog_signal_list_last[index] in [0, 1])):
            # tolerance means after number of tolerance unchanged watchdog,
            # state will be error
            if (self.watchdog_signal_list[index]
               == self.watchdog_signal_list_last[index]):
                self.tolerance_lists[index].append(1)
                self.tolerance_lists[index].pop(0)
            else:
                self.tolerance_lists[index].append(0)
                self.tolerance_lists[index].pop(0)
            if (self.tolerance_lists[index].count(0) > 0
               and len(self.tolerance_lists[index]) == self.tolerance_len):
                self.watchdog_state_list[index] = 30
            else:
                self.watchdog_state_list[index] = 60
                self.get_logger().error(
                    "Watchdog state error from node {} with tolerance {}."
                    .format(self.node_name_list[index],
                            self.tolerance_lists[index]))
        else:
            self.get_logger().error(
                "Unexpected watchdog signal from node {}"
                .format(self.node_name_list[index]))
            self.get_logger().error(
                "Unexpected watchdog signal: current {}|{} last {}|{}"
                .format(self.watchdog_signal_list[index],
                        type(self.watchdog_signal_list[index]),
                        self.watchdog_signal_list_last[index],
                        type(self.watchdog_signal_list_last[index])))
            self.watchdog_state_list[index] = 60


    # def trigger_state_monitor(self, data):
    #     """
    #     Turn right signal two times in 3 sec to set trigger state to ON
    #     The trigger state ON should be hold to at least 10s
    #     """
    #     msg_time = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9
    #     cur_time = self.get_clock().now().nanoseconds * 1e-9
    #     # Check if vehicle info message delayed in 0.2 sec
    #     transfer_time_diff_ = cur_time - msg_time
    #     if transfer_time_diff_ > 0.2:
    #         msg_delay_short = False
    #     else:
    #         msg_delay_short = True
    #     # Check if vehicle speed in allowed range 0-30 kph
    #     # with hysteresis +3 kph in 2 sec (2 sec = 200ms * 10cycles)
    #     vehicle_spd = data.v_ego
    #     if vehicle_spd > 33:
    #         self.cnt_hyst_spd = 0
    #         vehicle_spd_ready = False
    #     elif vehicle_spd < 30:
    #         self.cnt_hyst_spd = 0
    #         vehicle_spd_ready = True
    #     else:
    #         self.cnt_hyst_spd += 1
    #         if self.cnt_hyst_spd <= 10:
    #             vehicle_spd_ready = self.vehicle_spd_ready_last
    #         else:
    #             vehicle_spd_ready = False
    #     self.vehicle_spd_ready_last = vehicle_spd_ready

    #     # Check if acc pedal overrides trigger with 50% increment in 1 s
    #     # Cyclic time of cb is 20ms, then deque len 50 means 1 s
    #     if data.accel_pedal_position - min(self.acc_pedal_hist) < 50:
    #         no_pedal_overriden = True
    #     else:
    #         no_pedal_overriden = False

    #     if msg_delay_short and vehicle_spd_ready and no_pedal_overriden:
    #         # Calculate trigger state according to turn signal
    #         turn_signal_current = data.turning_light_switch
    #         if (turn_signal_current == 2
    #            and (turn_signal_current > self.turn_signal_last)):
    #             # rising edge detected
    #             if not self.rising_edge_time_last:
    #                 # first rising edge
    #                 self.rising_edge_time_last = self.get_clock().now()
    #                 self.rising_edge_count = 1
    #             elif ((self.get_clock().now() - self.rising_edge_time_last)
    #                     .nanoseconds/1e9 < 3):
    #                 # two rising edge signals in 3 sec
    #                 self.rising_edge_count += 1
    #                 if self.rising_edge_count >= 2:
    #                     self.trigger_time = self.get_clock().now()
    #                     self.rising_edge_time_last = self.trigger_time
    #                     self.trigger_state = 20
    #             else:
    #                 # two rising edge signals out 3 seconds
    #                 self.rising_edge_count = 1
    #                 self.rising_edge_time_last = self.get_clock().now()
    #         elif turn_signal_current == 2 and self.turn_signal_last == 2:
    #             # right turn signal is continously on
    #             if (self.trigger_state == 20
    #                 and self.trigger_time != self.rising_edge_time_last
    #                 and (self.get_clock().now() - self.trigger_time)
    #                     .nanoseconds/1e9 >= 60): ### temp: improve
    #                 # after trigger a non-right turn signal occurs in 10sec
    #                 self.trigger_state = 10
    #                 print("10s until off")
    #         else:
    #             # Other cases
    #             if (self.trigger_state == 20
    #                 and (self.get_clock().now() - self.trigger_time)
    #                     .nanoseconds/1e9 >= 60): ### temp: improve
    #                 # after trigger other signals occur outside 10 sec
    #                 self.trigger_state = 10
    #                 print("10s until off222222222")
    #         self.turn_signal_last = turn_signal_current
    #     else:
    #         self.trigger_state = 10
    #         print("other_cases")
    #         if not msg_delay_short:
    #             self.get_logger().warning(
    #                 "vehicle info msg delayed {} sec, trigger state OFF"
    #                 .format(transfer_time_diff_))
    #         elif not vehicle_spd_ready:
    #             self.get_logger().info(
    #                 "Vehile speed is {} kph out of range, trigger state OFF"
    #                 .format(data.v_ego))
    #         else:
    #             self.get_logger().info(
    #                 "acc pedal overrides with position {} percent"
    #                 .format(data.accel_pedal_position))
    #             self.get_logger().info(
    #                 "acc pedal positions {} in 1 sec"
    #                 .format(self.acc_pedal_hist))
    #     self.acc_pedal_hist.append(data.accel_pedal_position)



    def trigger_state_monitor(self, data):
        """
        Turn right signal two times in 3 sec to set trigger state to ON
        The trigger state ON should be hold to at least 10s
        """
        msg_time = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9
        cur_time = self.get_clock().now().nanoseconds * 1e-9
        # Check if vehicle info message delayed in 0.2 sec
        transfer_time_diff_ = cur_time - msg_time
        if transfer_time_diff_ > 0.2:
            msg_delay_short = False
        else:
            msg_delay_short = True
        # Check if vehicle speed in allowed range 0-30 kph
        # with hysteresis +3 kph in 2 sec (2 sec = 200ms * 10cycles)
        vehicle_spd = data.v_ego
        if vehicle_spd > 33:
            self.cnt_hyst_spd = 0
            vehicle_spd_ready = False
        elif vehicle_spd < 30:
            self.cnt_hyst_spd = 0
            vehicle_spd_ready = True
        else:
            self.cnt_hyst_spd += 1
            if self.cnt_hyst_spd <= 10:
                vehicle_spd_ready = self.vehicle_spd_ready_last
            else:
                vehicle_spd_ready = False
        self.vehicle_spd_ready_last = vehicle_spd_ready

        # Check if acc pedal overrides trigger with 50% increment in 1 s
        # Cyclic time of cb is 20ms, then deque len 50 means 1 s
        if data.accel_pedal_position - min(self.acc_pedal_hist) < 50:
            no_pedal_overriden = True
        else:
            no_pedal_overriden = False

        if msg_delay_short and vehicle_spd_ready and no_pedal_overriden:
            # Calculate trigger state according to turn signal
            turn_signal_current = data.turning_light_switch
            current_time = self.get_clock().now()

            if turn_signal_current == 2 and self.turn_signal_last == 0:
                # Rising edge detected (0 to 2)
                if not self.rising_edge_time_last:
                    # First rising edge
                    self.rising_edge_time_last = current_time
                    self.rising_edge_count = 1
                elif (current_time - self.rising_edge_time_last).nanoseconds / 1e9 < 3:
                    # Two rising edge signals within 3 sec
                    self.rising_edge_count += 1
                    if self.rising_edge_count >= 2:
                        self.trigger_state = 20
                        self.trigger_time = current_time
                        self.rising_edge_time_last = current_time
                        self.falling_edge_time = None
                else:
                    # Reset if more than 3 seconds have passed
                    self.rising_edge_time_last = current_time
                    self.rising_edge_count = 1

            elif turn_signal_current == 0 and self.turn_signal_last == 2 and self.rising_edge_count >= 2:
                # Falling edge detected (2 to 0)
                if self.trigger_state == 20:
                    self.falling_edge_time = current_time

            else:
                # Check if 10 seconds have passed since the last falling edge
                if self.falling_edge_time and self.trigger_state == 20:
                    if (current_time - self.falling_edge_time).nanoseconds / 1e9 >= 10:
                        self.trigger_state = 10
                        self.falling_edge_time = None
                        self.rising_edge_count = 0
                        self.rising_edge_time_last = None

            self.turn_signal_last = turn_signal_current
        else:
            self.trigger_state = 10
            print("other_cases")
            if not msg_delay_short:
                self.get_logger().warning(
                    "vehicle info msg delayed {} sec, trigger state OFF"
                    .format(transfer_time_diff_))
            elif not vehicle_spd_ready:
                self.get_logger().info(
                    "Vehile speed is {} kph out of range, trigger state OFF"
                    .format(data.v_ego))
            else:
                self.get_logger().info(
                    "acc pedal overrides with position {} percent"
                    .format(data.accel_pedal_position))
                self.get_logger().info(
                    "acc pedal positions {} in 1 sec"
                    .format(self.acc_pedal_hist))
        self.acc_pedal_hist.append(data.accel_pedal_position)

def main(args=None):
    rclpy.init(args=args)
    node = SystemManagerNode("node_system_manager")
    node_ego_fusion = EgoFusionNode("node_ego_fusion")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node_ego_fusion)
    executor.spin()
    # rclpy.spin(node)
    node.destroy_node()
    node_ego_fusion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
