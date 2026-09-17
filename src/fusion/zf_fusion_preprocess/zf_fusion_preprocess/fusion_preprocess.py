import copy
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from can_msgs.msg import CanDatas, RadarCamDatas, CanMsgData, CanSigData
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from .radar_postprocess import ObjectData, PostProcess5G4T, EPostStatus5G4T, PostProcessIPM, EProp5G4T, EPropIPM
# from radar_postprocess import dict_post_status_5G4T

class CommonUtil:
    @staticmethod
    def extract_number_from_string(string):
        number = ''.join(filter(str.isdigit, string))
        if number:
            return int(number)
        else:
            return -1

    @staticmethod
    def remove_number_from_string(string):
        return string.translate(str.maketrans('', '', '0123456789'))

    @staticmethod
    def remove_empty_strings_from_list(lst):
        return list(filter(lambda item: item != "", lst))

class ObjDataIpmLine:
    def __init__(self):
        self.__line_key_list = ["NumberOfLineObjects",
                        "RoadType",
                        "LaneChangeStatus",
                        "LaneChangeProbability",
                        "C0",
                        "C1",
                        "C2",
                        "C3",
                        "LineClass",
                        "Quality",
                        "ViewRange"
                        ]
        self._data_dic = {}
        self._lane_road_type_def = {
            0: "unknown",
            1: "highway",
            2: "inner city"
        }
        self._lane_change_status_def = {
            0: "No lane change",
            1: "Left lane change",
            2: "Right lane change"
        }
        self._line_class_def = {
            0: "undecided",
            1: "solid",
            2: "dashed",
            3: "double line crossable",
            4: "double line uncrossable",
            5: "multiple lines crossable",
            6: "multiple lines uncrossable",
            7: "Botts dots",
            8: "curb",
            9: "snow edge",
            10: "road edge",
            11: "virtual",
            12: "barrier",
            13: "cones"
        }
        self._line_quality_def = {
            0: "Low1_NoLineDetected",
            1: "Low2_InsufficientScore",
            2: "Medium_PredictedOrLowScore",
            3: "High"
        }

    def pack_ipm_line_data(self, data:CanDatas):
        self.clear_data_dic()
        for msg in data.msg_datas:
            self.extract_can_msg(msg)
        return data.header.stamp, self._data_dic

    def clear_data_dic(self):
        self._data_dic.clear()

    def get_data_dic(self):
        return self._data_dic

    def extract_can_msg(self, data:CanMsgData):
        if "Line" in data.msg_name:
            for sig in data.sig_datas:
                end_name = sig.sig_name.split("_")[-1]
                if end_name not in self.__line_key_list:
                    continue
                if end_name == "RoadType":
                    self._data_dic[sig.sig_name] = self._lane_road_type_def[int(sig.sig_data)]
                elif end_name == "LaneChangeStatus":
                    self._data_dic[sig.sig_name] = self._lane_change_status_def[int(sig.sig_data)]
                elif end_name == "LineClass":
                    self._data_dic[sig.sig_name] = self._line_class_def[int(sig.sig_data)]
                elif end_name == "Quality":
                    self._data_dic[sig.sig_name] = self._line_quality_def[int(sig.sig_data)]
                else:
                    self._data_dic[sig.sig_name] = sig.sig_data


class ObjDatasIpm:

    def __init__(self):
        # self.__key_list = ["Identifier",
        #                  "LongitudinalDistance",
        #                  "AbsoluteSpeed",
        #                  "OrientationAngle",
        #                  "ExistenceProbability",
        #                  "Class",
        #                  "DetectionStatus",
        #                  "MotionStatus",
        #                  "LateralDistance",
        #                  "Width",
        #                  "Lane",
        #                  "BrakeLight",
        #                  "RelativeVelocity",
        #                  "CutInCutOut",
        #                  "Length"]
        self.__key_dic = {
            "Identifier": "id",
            "LongitudinalDistance": "pos_x",
            "AbsoluteSpeed": "abs_spd",
            "OrientationAngle": "heading",
            "ExistenceProbability": "prob",
            "Class": "class",  #5
            "DetectionStatus": "status",  #6
            "MotionStatus": "motion",  #7
            "LateralDistance": "pos_y",
            "Width": "width",
            "Lane": "Lane",
            "BrakeLight": "BrakeLight",
            "RelativeVelocity": "rel_spd",
            "CutInCutOut": "CutInCutOut",
            "Length": "length",
            "Height": "height",
            "rel_spd_lat": "rel_spd_lat",
            "rel_spd_lon": "rel_spd_lon",
        }
        self.__key_list = self.__key_dic.keys()
        prop_len = len(self.__key_list)
        self.__index_dic = dict(zip(self.__key_list, range(prop_len)))
        self.__data_dic = {}

        # print(self.__data_dic)
        self.__dx = 3.83 + 1.54
        self.__dy = 0.0
        self.__y_coeff = -1.0
        self.__time_stamp = None
        self.__class_def = {
            0: "unknown",
            1: "truck",
            2: "car",
            3: "motorbike",
            4: "bicycle",
            5: "pedestrian",
            6: "undecided"
        }
        self.__detec_st = {0: "new", 1: "measured", 2: "predicted"}
        self.__motion_st = {
            0: "not defined",
            1: "stationary",  # standing in IPM dbc
            2: "passing in",
            3: "stopped",
            4: "passing out",
            5: "moving in",
            6: "moving out",
            7: "preceding",
            8: "moving oncoming",
            9: "crossing",
            10: "close cut-in",
            11: "moving unknow",
            12: "stopped unknow",
            13: "stopped crossing",
            14: "passing unknow"
        }
        self.__dimensions_set = {
            "pedestrian": (0.53, 0.5, 1.8),  # person
            "bicycle": (1.89, 0.5, 1.865),  # biycle
            "car": (4.075, 1.668, 1.474),  # car
            "motorbike": (2.025, 0.67, 1.0),  # motorcycle
            "truck": (7.2, 2.3, 2.7),  # truck
            "unknown": (0.5, 0.5, 0.5),
            "undecided": (0.5, 0.5, 0.5)
        }
        # self.dimensions_set = {
        #     0: (0.53, 0.5, 1.8),  # person
        #     1: (1.89, 0.5, 1.865),  # biycle
        #     2: (4.075, 1.668, 1.474),  # car
        #     3: (2.025, 0.67, 1.0),  # motorcycle
        #     5: (9.6, 2.5, 3.45),  # bus
        #     7: (7.2, 2.3, 2.7)  # truck
        # }
        self.__line_process = ObjDataIpmLine()
        self.post_process_IPM = PostProcessIPM()

    def set_detaxy(self, dx, dy):
        self.__dx = dx
        self.__dy = dy

    def col_names(self):
        return list(self.__key_dic.values())

    def IPM_obj_class(self):
        return self.__class_def

    def IPM_detection_st(self):
        return self.__detec_st

    def IPM_motion_st(self):
        self.__motion_st

    def pack_data(self, data: CanDatas):
        tdata_dic = {}
        self.__line_process.clear_data_dic()
        for msg in data.msg_datas:
            # FC_Video_Object_09_D
            obj_id = CommonUtil.extract_number_from_string(msg.msg_name)
            #
            if (obj_id == -1):
                self.__line_process.extract_can_msg(msg)
                continue
            else:
                if not tdata_dic.get(obj_id):
                    tdata_dic[obj_id] = [None] * len(self.__key_list)
            # print(msg.msg_name, " obj_id is", obj_id)
            for sig in msg.sig_datas:
                end_name = sig.sig_name.split("_")[
                    -1]  # TODO FC_Obj00_MessageCounter_C
                index = self.__index_dic.get(end_name)
                # print(sig.sig_name, ", ", end_name, " obj_id is", obj_id, ", index: ", index, ", data: ",sig.sig_data)
                if index is not None:
                    # tindex = self.__data_dic.get(obj_id)
                    # print("--------", end_name, "--------")
                    # print("--------", self.__data_dic[obj_id])
                    # print("--------", self.__data_dic[obj_id][index])
                    if end_name == "Class":
                        tdata_dic[obj_id][index] = self.__class_def[int(
                            sig.sig_data)]
                    elif end_name == "DetectionStatus":
                        tdata_dic[obj_id][index] = self.__detec_st[int(
                            sig.sig_data)]
                    elif end_name == "MotionStatus":
                        tdata_dic[obj_id][index] = self.__motion_st[int(
                            sig.sig_data)]
                    elif end_name == "LongitudinalDistance":
                        tdata_dic[obj_id][index] = sig.sig_data + self.__dx
                    elif end_name == "LateralDistance":
                        tdata_dic[obj_id][
                            index] = self.__y_coeff * sig.sig_data + self.__dy
                    elif end_name == "Identifier":
                        tdata_dic[obj_id][0] = sig.sig_data
                        # print(sig.sig_name, ", ", end_name, " obj_id is", obj_id, ", index: ", index, ", data: ",sig.sig_data)
                    else:
                        tdata_dic[obj_id][index] = sig.sig_data
        # self.__time_stamp = str(data.header.stamp.sec) + "." + str(data.header.stamp.nanosec).zfill(9)
        index_heading = self.__index_dic.get("OrientationAngle")
        index_abs_spd = self.__index_dic.get("AbsoluteSpeed")
        index_rel_spd = self.__index_dic.get("RelativeVelocity")
        index_width = self.__index_dic.get("Width")
        index_length = self.__index_dic.get("Length")
        index_height = self.__index_dic.get("Height")
        index_class = self.__index_dic.get("Class")
        self.__time_stamp = data.header.stamp
        tdata_dic = {
            int(value[0]): value
            for key, value in tdata_dic.items()
            if (value[0] is not None and int(value[0]) != 0)
        }
        self.__data_dic = tdata_dic
        for key, value in self.__data_dic.items():
            abs_spd = value[index_abs_spd]
            rel_spd = value[index_rel_spd]
            heading = value[index_heading]
            if abs(value[index_abs_spd]) <= 0.5:
                rel_spd_lon = rel_spd
                rel_spd_lat = 0.0
            else:
                rel_spd_lon = abs(rel_spd) * math.cos(heading)
                rel_spd_lat = abs(rel_spd) * math.sin(heading)
            if rel_spd >= 0:
                rel_spd_lon = abs(rel_spd_lon)
            else:
                rel_spd_lon = -abs(rel_spd_lon)
            value[-2] = rel_spd_lat
            value[-1] = rel_spd_lon
            if value[index_length] <= 0.01:
                value[index_length] = self.__dimensions_set[
                    value[index_class]][0]
            if value[index_width] <= 0.01:
                value[index_width] = self.__dimensions_set[
                    value[index_class]][1]
        return self.__time_stamp, tdata_dic, self.__line_process.get_data_dic()

    def get_prop_index(self, prop_name: str):
        return self.__index_dic.get(prop_name)

    def get_time_stamp(self):
        return self.__time_stamp

    # 1: [0, 0, 0, ...]; 2: [0, 0, 0, ...]; ...
    def get_data(self):  #-> dict[int, list]
        return self.__data_dic

    def dict2strs(self):
        str_list = []
        val_list = self.__data_dic.values()
        for v in val_list:
            str_prop = self.list2str(v, "14")
            str_list.append(str_prop)
        return str_list

    def list2str(self, v, source):
        prop_list = [v[EPropIPM.CLASS],
                     source,
                     source,
                     str(v[EPropIPM.pos_x]),
                     str(v[EPropIPM.pos_y]),
                     str(v[EPropIPM.width]),
                     str(v[EPropIPM.length]),
                     str(v[EPropIPM.heading]),
                     str(v[EPropIPM.rel_spd_lon]),
                     str(v[EPropIPM.rel_spd_lat]),
                     str(v[EPropIPM.id])]
        str_prop = " ".join(prop_list)
        return str_prop

    def get_frame_data(self, data: CanDatas):
        self.pack_data(data)
        ipm_objs = self.dict2strs()
        timestampf = self.__time_stamp.sec + self.__time_stamp.nanosec / 1e9
        t_obj = ObjectData(self.__data_dic, timestampf)
        obj = self.post_process_IPM.add_object(t_obj)
        data_dict = {}
        cols = self.col_names()
        cols.append("post_status")
        ipm_objs_pp = []
        for key, val in obj.get_data().items():
            ipm_objs_pp.append(self.list2str(val, "15"))
            data_dict[key] = dict(zip(cols, val))
        return data_dict, ipm_objs, ipm_objs_pp


class ObjDatas5G4T:

    def __init__(self):
        # self.__key_list = ["ID_A",
        #                  "rel_Lat_Speed",
        #                  "rel_Long_Speed",
        #                  "rel_Lat_Pos",
        #                  "rel_Long_Pos",
        #                  "TrackingStatus",
        #                  "Motion_Class",
        #                  "Length",
        #                  "Width",
        #                  "Lifetime",
        #                  "ClassConfidence",
        #                  "Class",
        #                  "Quality"]
        self.__key_dic = {
            "ID_A": "id",
            "rel_Lat_Speed": "rel_spd_lat",
            "rel_Long_Speed": "rel_spd_lon",
            "rel_Lat_Pos": "pos_y",
            "rel_Long_Pos": "pos_x",
            "TrackingStatus": "status",
            "Motion_Class": "motion",
            "Length": "length",
            "Width": "width",
            "Lifetime": "lifetime",
            "ClassConfidence": "class_confidence",
            "Class": "class",
            "Quality": "prob",
            "heading": "heading",
            "rel_spd": "rel_spd",
        }
        self.__key_list = self.__key_dic.keys()
        self.__prop_len = len(self.__key_list)
        self.__index_dic = dict(zip(self.__key_list, range(self.__prop_len)))
        self.__data_dic = {}
        self.__data_status = {}
        self.__time_stamp = None
        self.__class_def = {}
        self.__class_def = self.__class_def.fromkeys(range(16),
                                                     "not_available")
        self.__class_def[0] = "unknown"
        self.__class_def[1] = "pedestrian"
        self.__class_def[2] = "2W"
        self.__class_def[3] = "car"
        self.__class_def[4] = "truck_or_bus"
        self.__class_def[5] = "VRU"
        self.__class_def[6] = "non_VRU"
        self.__detec_st = {0: "new", 1: "measured", 2: "predicted"}
        self.__dic_radar_status = {
            0: "Initializing",
            1: "Fully Operational",
            2: "Performance Limited",
            3: "Temporary Fault - Pending Recovery",
            4: "Permanent Error - Reset to Recover",
            5: "Falling Asleep",
            6: "Shutting Down",
            7: "Power Save / Testbench"
        }
        self.__motion_st = {}
        self.__motion_st = self.__motion_st.fromkeys(range(8), "not_available")
        self.__motion_st[0] = "not defined"
        self.__motion_st[1] = "stationary"
        self.__motion_st[2] = "moving"
        self.__motion_st[3] = "stopped"
        self.post_process_5G4T = PostProcess5G4T()

    def col_names(self):
        return list(self.__key_dic.values())

    def obj_class_5G4T(self):
        return self.__class_def

    def detection_st__5G4T(self):
        return self.__detec_st

    def motion_st__5G4T(self):
        self.__motion_st

    def pack_data(self, data: CanDatas):
        self.__data_dic.clear()
        self.__data_status = ""
        for msg in data.msg_datas:
            if (msg.msg_name == "SRR_S5_C0"):
                for sig in msg.sig_datas:
                    end_name = sig.sig_name.split("_")[-1]
                    if end_name == "Status":
                        self.__data_status = self.__dic_radar_status[int(
                            sig.sig_data)]
                        break
                continue
            if (msg.msg_name == "SRR_S1_B0"):
                for sig in msg.sig_datas:
                    end_name = sig.sig_name.split("_")[-1]
                    if end_name == "Status":
                        self.__data_status_B0 = self.__dic_radar_status[int(
                            sig.sig_data)]
                        break
                continue
            obj_id = msg.msg_name[:-1]
            if not self.__data_dic.get(obj_id):
                self.__data_dic[obj_id] = [None] * self.__prop_len
            for sig in msg.sig_datas:
                t_name = CommonUtil.remove_number_from_string(sig.sig_name)
                end_name_list = CommonUtil.remove_empty_strings_from_list(
                    t_name.split("_")[3:])
                end_name = "_".join(end_name_list)
                index = self.__index_dic.get(end_name)
                if index is not None:
                    if end_name == "Class":
                        self.__data_dic[obj_id][index] = self.__class_def[int(
                            sig.sig_data)]
                    elif end_name == "TrackingStatus":
                        self.__data_dic[obj_id][index] = self.__detec_st[int(
                            sig.sig_data)]
                    elif end_name == "Motion_Class":
                        self.__data_dic[obj_id][index] = self.__motion_st[int(
                            sig.sig_data)]
                    elif end_name == "rel_Long_Pos":
                        self.__data_dic[obj_id][index] = sig.sig_data + 3.83
                    else:
                        self.__data_dic[obj_id][index] = sig.sig_data
        # self.__time_stamp = str(data.header.stamp.sec) + "." + str(data.header.stamp.nanosec).zfill(9)
        for key, value in self.__data_dic.items():
            # value[1] = 3
            # value[2] = 4
            # df["heading"] = np.arctan(df["rel_spd_lat"]/df["rel_spd_lon"])
            if value[1] is None or value[2] is None:
                # print("--------------------None")
                # print(self.__data_dic)
                continue
            value[-2] = math.atan2(value[1], value[2])
            # df["rel_spd"] = df["rel_spd_lon"] / np.cos(df["heading"])
            value[-1] = math.sqrt(value[1]**2 + value[2]**2)
            value.append(EPostStatus5G4T.INIT)
        self.__time_stamp = data.header.stamp
        self.__data_dic = {
            int(value[0]): value
            for key, value in self.__data_dic.items()
            if (value[0] is not None and int(value[0]) != 0)
        }
        return self.__time_stamp, self.__data_dic, self.__data_status

    def get_prop_index(self, prop_name: str):
        return self.__index_dic.get(prop_name)

    def get_time_stamp(self):
        return self.__time_stamp

    # 1: [0, 0, 0, ...]; 2: [0, 0, 0, ...]; ...
    def get_data(self):  # -> dict[int, list]
        return self.__data_dic

    def dict2strs(self):
        str_list = []
        val_list = self.__data_dic.values()
        for v in val_list:
            str_prop = self.list2str(v, "12")
            str_list.append(str_prop)
        return str_list

    def list2str(self, v, source):
        prop_list = [v[EProp5G4T.CLASS],
                     source,
                     source,
                     str(v[EProp5G4T.pos_x]),
                     str(v[EProp5G4T.pos_y]),
                     str(v[EProp5G4T.width]),
                     str(v[EProp5G4T.length]),
                     str(v[EProp5G4T.heading]),
                     str(v[EProp5G4T.rel_spd_lon]),
                     str(v[EProp5G4T.rel_spd_lat]),
                     str(v[EProp5G4T.id])]
        str_prop = " ".join(prop_list)
        return str_prop


    def get_frame_data(self, data: CanDatas, ego_angle): # ego_angle: rad
        self.pack_data(data)
        srr_objs = self.dict2strs()
        timestampf = self.__time_stamp.sec + self.__time_stamp.nanosec / 1e9
        obj_data_5G4T = ObjectData(self.__data_dic, timestampf)
        self.post_process_5G4T.set_ego_state_angle(ego_angle)
        revised_obj = self.post_process_5G4T.add_object_data(obj_data_5G4T)
        tdata = revised_obj.get_data()
        filter_list = [
            EPostStatus5G4T.INIT, EPostStatus5G4T.COLLISION_TRAILER,
            EPostStatus5G4T.DECT_STOP
        ]
        data_dict = {}
        cols = self.col_names()
        cols.append("post_status")
        srr_objs_pp = []
        for key, val in tdata.items():
            if val[-1] in filter_list:
                srr_objs_pp.append(self.list2str(val, "13"))
                data_dict[key] = dict(zip(cols, val))
        return data_dict, self.__data_status, srr_objs, srr_objs_pp
