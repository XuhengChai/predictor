# -*- coding: utf-8 -*-
"""
Created on Tue Nov 07 15:27:28 2023

@author: Z0030883
"""
from typing import SupportsFloat as Numeric

import rclpy  # ROS2 Python API library
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from std_msgs.msg import String


# from python folder under package import filename
from can_msgs.msg import CanMsgData
from can_msgs.msg import CanDatas
from can_msgs.msg import CanSigData

from interface.msg import DetectedEgoInfo
from interface.msg import VehicleInfo
from interface.msg import NodeState
from ego_state.ego_state_lib import Ego_State
from ego_state.unit_converter import UnitConverter
from ego_state.config import Config

# from ego_state import ego_state_right_fisheye


class EgoStateNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.config = Config()
        self.egostate = Ego_State()
        self.egostate_msg = VehicleInfo()
        self.node_state_msg = NodeState()
        self.unit_conv = UnitConverter()
        # self.egostate_cv = ego_state_right_fisheye.FindEgoStateAngle()

        self.msg_update_funs = {
            "EBC5_EBS": self.update_EBC5_EBS,
            "EC1": self.update_EC1,
            "EEC2": self.update_EEC2,
            "EEC3": self.update_EEC3,
            "ETC1": self.update_ETC1,
            "ETC2": self.update_ETC2,
            "HRW": self.update_HRW,
            "CVW_AMT": self.update_CVW_AMT,
            "CVW_EBS": self.update_CVW_EBS,
            "VDC2": self.update_VDC2,
            "TCO1": self.update_TCO1,
            "XBR_AEBS": self.update_XBR_AEBS,
            "LD": self.update_LD,
            "OEL": self.update_OEL,
            "EBC1_EBS": self.update_EBC1_EBS,
            "TRD_C0": self.update_TRD,
            "DM1_TCM": self.update_DM1_TCM,
            "DM1_ENG": self.update_DM1_ENG,
            "DM1_EBS": self.update_DM1_EBS,
        }
        self.signals_index = {}

        self.my_callback_group1 = MutuallyExclusiveCallbackGroup()
        self.my_callback_group5 = MutuallyExclusiveCallbackGroup()
        self.my_callback_group2 = ReentrantCallbackGroup()
        self.my_callback_group3 = MutuallyExclusiveCallbackGroup()
        self.my_callback_group4 = MutuallyExclusiveCallbackGroup()
        # self.my_callback_group5 = MutuallyExclusiveCallbackGroup()

        # Define a timer object for periodically calling the timer_callback.
        self.timer = self.create_timer(
            self.config.ts,
            self.timer_callback,
            callback_group=self.my_callback_group1)  # self.config.ts
        self.timer_state = self.create_timer(
            0.2,
            self.timer_state_callback,
            callback_group=self.my_callback_group5)

        qos_profile_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=500)
        qos_profile_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=500)
        # Define 2 subscribers object for invoking the listener_callback upon
        # receiving messages, one for PCAN and one for RCAN.
        self.canmsg_sub = self.create_subscription(
            CanMsgData,
            "/can0/hal/from_can_msg_data",
            self.vcan_listener_callback_new,
            qos_profile_best_effort,
            callback_group=self.my_callback_group2)

        # self.radarmsg_sub = self.create_subscription(
        #     CanMsgData,
        #     "/can1/hal/from_can_msg_data",
        #     self.rcan_listener_callback,
        #     qos_profile_best_effort,
        #     callback_group=self.my_callback_group3)

        self.detect_msg_sub = self.create_subscription(
            DetectedEgoInfo,
            "/detected_info", 
            self.detect_listener_callback,
            qos_profile_best_effort,
            callback_group=self.my_callback_group4)
        
        # self.can_timeout_sub = self.create_subscription(
        #     String,
        #     "/can0/can_msg_status",
        #     self.can_timeout_callback,
        #     qos_profile_best_effort,
        #     callback_group=self.my_callback_group5)

        # self.imagemsg_sub = self.create_subscription(
        #     CompressedImage, "/cam2/camera/image_raw/compressed",
        #     self.image_listener_callback, qos_profile)

        # Define a publisher object for publishing egostate messages.
        self.egostate_pub = self.create_publisher(VehicleInfo, "/vehicle_info",
                                                  qos_profile_best_effort)
        self.egostate_pub_sync = self.create_publisher(
            CanDatas, "can0/hal/from_packed_radar_data",
            qos_profile_best_effort)
        self.node_state_pub = self.create_publisher(NodeState, "/ego_state",
                                                    qos_profile_reliable)

        self.node_state_msg.node_state = 10
        self.node_state_msg.watchdog_signal = 0

        self.record_dic = {}
        self.record_list = [
            'm', 'v_ego', 'gear_ratio', 'a_ecoroll', 'engine2wheel_ratio',
            'max_acc', 'tractor_yawrate', 'artic_angle', 'artic_angle_confid',
            'trailer_wheelbase_wb_save', 'trailer_len', 'trailer_len_confid',
            'steering_angle', 'is_wheel_turn', 'is_forward_drive'
        ]
        self.record_dic_ego_sync = {
            'v_ego': self.egostate_msg.v_ego,
            'tractor_yawrate': self.egostate_msg.tractor_yawrate,
            'artic_angle': self.egostate_msg.artic_angle,
            'artic_angle_confid': self.egostate_msg.artic_angle_confid,
            'trailer_wheelbase_wb_save':
            self.egostate_msg.trailer_wheelbase_wb_save,
            'trailer_len': self.egostate_msg.trailer_len,
            'trailer_len_confid': self.egostate_msg.trailer_len_confid,
            'steering_angle': self.egostate_msg.steering_angle,
            'is_wheel_turn': self.egostate_msg.is_wheel_turn,
            'is_forward_drive': self.egostate_msg.is_forward_drive,
            'turning_light_switch': self.egostate_msg.turning_light_switch,
            'acceleration': self.egostate_msg.acceleration
        }
        self.ego_sync_datas = CanDatas()
        
        self.time_out_list = []  # EEC2 EEC3 ETC1 ETC2 HRW CVW_AMT CVW_EBS VDC2 TCO1 XBR_AEB OEL EBC1_EBS

        self.egostate_sig_list = ['reference_torque',
                                  'max_available_torque',
                                  'friction_torque',
                                  'tsmo_rpm',
                                  'tsmi_rpm',
                                  'shift_process',
                                  'gear_ratio',
                                  'current_gear',
                                  'hrw_speed_fl',
                                  'hrw_speed_fr',
                                  'weight_amt',
                                  'weight_ebs',
                                  'tractor_yawrate',
                                  'wheel_angle',
                                  'tach_speed'
                                  ]
        self.egostate_msg_sig_list = ['xbr_deceleration_limit',
                                      'accel_pedal_position',
                                      'acceleration',
                                      'turning_light_switch',
                                      'brake_pedal_position',
                                      'pri_xbr_aeb',
                                      'amber_lamp_ebs',
                                      'red_lamp_ebs',
                                      'amber_lamp_eng',
                                      'red_lamp_eng',
                                      'amber_lamp_tcm',
                                      'red_lamp_tcm'
                                      ]
        
    def timer_callback(self):
        self.egostate.get_tractor_yawrate()
        self.egostate.update()
        self.egostate.get_eng2whl_ratio()
        self.egostate.get_ecoroll_acc()
        self.egostate.get_max_acceleration()
        self.update_msg()
        # self.save_record_data()
        self.egostate_pub.publish(self.egostate_msg)
        self.egostate_pub_sync.publish(self.ego_sync_datas)

    def timer_state_callback(self):
        self.node_state_pub.publish(self.node_state_msg)
        self.update_state()

    def update_msg(self):
        # The following is for updating egostate class parameter to egostate
        # message, which is used to publish.
        self.egostate_msg.m = self.egostate.veh_weight
        self.egostate_msg.weight_amt = self.egostate.weight_amt
        self.egostate_msg.weight_ebs = self.egostate.weight_ebs
        self.egostate_msg.v_ego = self.egostate.veh_speed
        # print(self.egostate.veh_speed)
        self.egostate_msg.hrw_speed_fl = self.egostate.hrw_speed_fl
        self.egostate_msg.tsmo_rpm = self.egostate.tsmo_rpm
        self.egostate_msg.tsmo_speed = self.egostate.tsmo_speed
        self.egostate_msg.tsmi_rpm = self.egostate.tsmi_rpm
        self.egostate_msg.tsmi_speed = self.egostate.tsmi_speed
        self.egostate_msg.tach_speed = self.egostate.tach_speed
        self.egostate_msg.gear_ratio = self.egostate.gear_ratio

        self.egostate_msg.a_ecoroll = self.egostate.a_ecoroll
        self.egostate_msg.engine2wheel_ratio = self.egostate.eng2whl_ratio
        self.egostate_msg.max_acc = self.egostate.max_acc
        self.egostate_msg.tractor_yawrate = self.egostate.tractor_yawrate
        self.egostate_msg.artic_angle = self.egostate.artic_angle
        self.egostate_msg.artic_angle_vd = self.egostate.artic_angle_vd
        self.egostate_msg.artic_angle_confid = self.egostate.artic_angle_confid
        self.egostate_msg.artic_angle_confid_vd = self.egostate.artic_angle_confid_vd
        (self.egostate_msg.trailer_wheelbase_wb_save) = (
            self.egostate.trailer_wheelbase_wb_save)
        self.egostate_msg.trailer_len = self.egostate.trailer_len
        self.egostate_msg.trailer_len_confid = self.egostate.trailer_len_confid

        self.egostate_msg.reference_torque = self.egostate.reference_torque
        self.egostate_msg.friction_torque = self.egostate.friction_torque
        self.egostate_msg.steering_angle = self.egostate.wheel_angle

        self.egostate_msg.forward_drive_distance = (
            self.egostate.forward_drive_distance)

        self.egostate_msg.is_wheel_turn = bool(
            abs(self.egostate.wheel_angle) >
            self.egostate.wheel_angle_strght_uplimit)

        self.egostate_msg.is_forward_drive = bool(
            self.egostate.forward_drive_distance >
            self.egostate.forward_drive_distance_lwlimit)

        self.egostate_msg.current_gear = self.egostate.current_gear
        self.egostate_msg.shift_process = self.egostate.shift_process
        self.egostate_msg.header.stamp = self.get_clock().now().to_msg()


        # update ego_sync msg: CanDatas
        self.ego_sync_datas.header.stamp = self.egostate_msg.header.stamp
        t_msg_data = CanMsgData()
        for name, val in self.record_dic_ego_sync.items():
            t_sig = CanSigData()
            t_sig.sig_name = name
            t_sig.sig_data = float(getattr(self.egostate_msg, name))
            t_msg_data.sig_datas.append(t_sig)
        self.ego_sync_datas.msg_datas = [t_msg_data]
        self.ego_sync_datas.data_size = len(self.ego_sync_datas.msg_datas)

    # def save_record_data(self):
    #     for item in self.record_list:
    #         self.record_dic[item] = getattr(self.egostate_msg, item)
    #     with open(
    #             '/home/nvidia/ata_ws_20240415/src/ego_state/ego_state/egostate_record.txt',
    #             'a') as f:
    #         lines = [
    #             "time_stamp,",
    #             str(self.egostate_msg.header.stamp.sec), ".",
    #             str(self.egostate_msg.header.stamp.nanosec).zfill(9), ","
    #         ]
    #         f.writelines(lines)
    #         for key, val in self.record_dic.items():
    #             print(key, val, file=f, end=',', sep=',')
    #         f.write('\n')

    def update_state(self):
        self.egostate.veh_weight_downgrade = 0 # bypass vehicle weight estimation failure at start
        downgrade_level = sum([
            self.egostate.veh_speed_downgrade,
            self.egostate.veh_weight_downgrade,
            self.egostate.artic_angle_downgrade,
        ])
        # Publish the customized egostate ROS message.
        if downgrade_level in range(1, 3):
            self.node_state_msg.node_state = 50
        elif downgrade_level == 3:
            self.node_state_msg.node_state = 60
        else:
            self.node_state_msg.node_state = (30 
                                              if sum(self.time_out_list) == 0
                                              else 50)


        # the following line switches the watchdog_state between 0 and 1
        self.node_state_msg.watchdog_signal = (
            1 - self.node_state_msg.watchdog_signal)
        self.node_state_msg.header.stamp = self.get_clock().now().to_msg()

    #  This is a CAN message and signal update template function.
    def update_can_msg(self, msg, egostate_param: Numeric, dbc_sig_name: str):
        # signal_value = [sig.sig_data for sig in msg.sig_datas if
        #                 sig.sig_name == dbc_sig_name]
        signal_index_temp = self.signals_index.get(msg.msg_name, dbc_sig_name)
        if signal_index_temp:
            return type(egostate_param)(msg.sig_datas[signal_index_temp].sig_data)
        else:
            for sig in msg.sig_datas:
                if sig.sig_name == dbc_sig_name:
                    signal_value = sig.sig_data
                    signal_index = msg.sig_datas.index(sig)
                    self.signals_index[msg.msg_name, dbc_sig_name] = signal_index
                    return type(egostate_param)(signal_value)
            print(f"No {dbc_sig_name} found")
        return egostate_param

    def update_EBC5_EBS(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate_msg.xbr_deceleration_limit,
                                 "XBRAccelerationLimit")
        self.egostate_msg.xbr_deceleration_limit = new_sig

    def update_EC1(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.reference_torque,
                                 "EngReferenceTorque")
        self.egostate.reference_torque = new_sig

    def update_EEC2(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.max_available_torque,
                                 "ActMaxAvailEngPercentTorque")
        self.egostate.max_available_torque = new_sig

        new_sig = self.update_can_msg(msg, self.egostate_msg.accel_pedal_position,
                                 "AccelPedalPos1")
        self.egostate_msg.accel_pedal_position = new_sig

    def update_EEC3(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.friction_torque,
                                 "NominalFrictionPercentTorque")
        self.egostate.friction_torque = new_sig

    def update_ETC1(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.tsmo_rpm,
                                 "TransOutputShaftSpeed")
        self.egostate.tsmo_rpm = new_sig

        if self.egostate.shift_process:
            self.egostate.tsmi_rpm = self.egostate.tsmi_rpm_prev
        else:
            new_sig = self.update_can_msg(msg, self.egostate.tsmi_rpm,
                                 "TransInputShaftSpeed")
            self.egostate.tsmi_rpm = new_sig

        new_sig = self.update_can_msg(msg, self.egostate.shift_process,
                                 "TransShiftInProcess")
        self.egostate.shift_process = new_sig

    def update_ETC2(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.gear_ratio,
                                 "TransActualGearRatio")
        # self.get_logger().info("# !!!ETC2 data is {:.4f} ".format(new_sig))
        self.egostate.gear_ratio = new_sig

        new_sig = self.update_can_msg(msg, self.egostate.current_gear,
                                 "TransCurrentGear")
        self.egostate.current_gear = new_sig

    def update_HRW(self, msg):
        # print("HRW comming")
        new_sig = self.update_can_msg(msg, self.egostate.hrw_speed_fl,
                                 "FrontAxleLeftWheelSpeed")
        self.egostate.hrw_speed_fl = new_sig

        new_sig = self.update_can_msg(msg, self.egostate.hrw_speed_fr,
                                 "FrontAxleRightWheelSpeed")
        self.egostate.hrw_speed_fr = new_sig

    def update_CVW_AMT(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.weight_amt,
                                 "GrossCombinationVehicleWeight")
        self.egostate.weight_amt = new_sig

    def update_CVW_EBS(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.weight_ebs,
                                 "GrossCombinationVehicleWeight")
        self.egostate.weight_ebs = new_sig

    def update_VDC2(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.tractor_yawrate, "YawRate")
        self.egostate.tractor_yawrate = new_sig

        new_sig = self.update_can_msg(msg, self.egostate.wheel_angle,
                                 "SteerWheelAngle")
        self.egostate.wheel_angle = new_sig

        new_sig = self.update_can_msg(msg, self.egostate_msg.acceleration,
                                 "LongitudinalAcceleration")
        self.egostate_msg.acceleration = new_sig

    def update_TCO1(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.tach_speed,
                                 "TachographVehicleSpeed")
        self.egostate.tach_speed = new_sig

    def update_TRD(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate.artic_angle_sr,
                                 "FC_Obj11_LeftAngle")
        self.egostate.artic_angle_sr = new_sig * self.unit_conv.deg2rad
        print(new_sig)
        new_sig = self.update_can_msg(msg, self.egostate.artic_angle_confid_sr,
                                 "TrailerKinkAngleQuality")
        self.egostate.artic_angle_confid_sr = new_sig / 100

        new_sig = self.update_can_msg(msg, self.egostate.trailer_len_sr,
                                 "TrailerLength")
        self.egostate.trailer_len_sr = new_sig * self.unit_conv.deg2rad

        new_sig = self.update_can_msg(msg, self.egostate.trailer_len_confid_sr,
                                 "TrailerLengthQuality")
        self.egostate.trailer_len_confid_sr = new_sig / 100

        self.egostate.len_sr_update()


#  The following second argument is different from the above, because there is
#  no corresponding paramter in egostate Class, so the signal is directly
#  forwarded to the ROS message, this is useful for only CAN forward.

    def update_LD(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate_msg.turning_light_right,
                                 "RightTurnSignalLightsData")
        self.egostate_msg.turning_light_right = new_sig

    def update_OEL(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate_msg.turning_light_switch,
                                 "TurnSignalSwitch")
        # startpoint_begin_ = self.get_clock().now()
        # 1.Calculate FPS
        # curr_time_stamp = self.get_clock().now()
        # elapsed_time = (curr_time_stamp - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        # self.get_logger().info('[OEL]: data is %.4f elapsed_time: "%.4f"' %(new_sig, elapsed_time))
        self.egostate_msg.turning_light_switch = new_sig

    def update_EBC1_EBS(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate_msg.brake_pedal_position,
                                 "BrakePedalPos")
        self.egostate_msg.brake_pedal_position = new_sig

    def update_XBR_AEBS(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate_msg.pri_xbr_aeb,
                                 "XBRPriority")
        self.egostate_msg.pri_xbr_aeb = new_sig

    def update_DM1_EBS(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate_msg.amber_lamp_ebs,
                                 "AmberWarningLampStatus")
        self.egostate_msg.amber_lamp_ebs = new_sig

        new_sig = self.update_can_msg(msg, self.egostate_msg.red_lamp_ebs,
                                 "RedStopLampState")
        self.egostate_msg.red_lamp_ebs = new_sig

    def update_DM1_ENG(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate_msg.amber_lamp_eng,
                                 "AmberWarningLampStatus")
        self.egostate_msg.amber_lamp_eng = new_sig

        new_sig = self.update_can_msg(msg, self.egostate_msg.red_lamp_eng,
                                 "RedStopLampState")
        self.egostate_msg.red_lamp_eng = new_sig

    def update_DM1_TCM(self, msg):
        new_sig = self.update_can_msg(msg, self.egostate_msg.amber_lamp_tcm,
                                 "AmberWarningLampStatus")
        self.egostate_msg.amber_lamp_tcm = new_sig

        new_sig = self.update_can_msg(msg, self.egostate_msg.red_lamp_tcm,
                                 "RedStopLampState")
        self.egostate_msg.red_lamp_tcm = new_sig

    def vcan_listener_callback(self, data):
        # 判断输入是否在列表中，并执行相应的操作
        if data.msg_name in self.msg_update_funs.keys():
            self.msg_update_funs[data.msg_name](data)
            
    def vcan_listener_callback_new(self, data):
        # startpoint_begin_ = self.get_clock().now()
        # curr_time_stamp = self.get_clock().now()
        # elapsed_time = (curr_time_stamp - Time.from_msg(data.header.stamp)).nanoseconds * 1e-9
        # self.get_logger().info('[CAN]: elapsed_time: "%.4f"' %(elapsed_time))
        # 判断输入是否在列表中，并执行相应的操作
        for sig in data.sig_datas:
            if sig.sig_name in self.egostate_sig_list:
                setattr(self.egostate, sig.sig_name, type(getattr(self.egostate, sig.sig_name))(sig.sig_data))
            # elif sig.sig_name == "tsmi_rpm":
            #     if self.egostate.shift_process:
            #         self.egostate.tsmi_rpm = self.egostate.tsmi_rpm_prev
            #     else:
            #         self.egostate.tsmi_rpm = type(self.egostate.tsmi_rpm)(sig.sig_data)
            else:
                setattr(self.egostate_msg, sig.sig_name, type(getattr(self.egostate_msg, sig.sig_name))(sig.sig_data))
            

    def rcan_listener_callback(self, data):
        if data.msg_name == "FC_Video_Object_11_B":  # TRD
            self.msg_update_funs[data.msg_name](data)

    def detect_listener_callback(self, data):
        self.egostate.artic_angle_cv = (data.articulation_angle *
                                        self.unit_conv.deg2rad) * -1
        self.egostate_msg.artic_angle_cv = self.egostate.artic_angle_cv
        self.egostate.artic_angle_confid_cv = data.confidence
        self.egostate_msg.artic_angle_confid_cv = self.egostate.artic_angle_confid_cv
        
    # def can_timeout_callback(self, data):

    #     self.time_out_list = [int(x) for x in data.data]
    # def image_listener_callback(self, data):
    #     if (self.egostate.wheel_angle
    #        > self.egostate.wheel_angle_strght_uplimit):
    #         self.egostate_cv._state = "Disable"
    #         self.egostate_cv.reset_init()
    #     if ((self.egostate.forward_drive_distance
    #        > self.egostate.forward_drive_distance_lwlimit)
    #        or (self.egostate_cv._state == "Enable")):
    #         self.egostate_cv.angle_cal(data)
    #         self.egostate_cv._state = "Enable"
    #     self.egostate.artic_angle_cv = (self.egostate_cv._cur_angle
    #                                     * self.unit_conv.deg2rad)
    #     self.egostate.artic_angle_confid_cv = self.egostate_cv._confidence

def main(args=None):
    rclpy.init(args=args)
    node = EgoStateNode("node_es")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
