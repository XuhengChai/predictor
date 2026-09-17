/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/**
 * @file
 * @brief Defines the hal radar data preprocess policy.
 * @return
 */

#ifndef ZF_HAL_RADAR_PREPROCESS_H_
#define ZF_HAL_RADAR_PREPROCESS_H_

#include <mutex>
// #include <deque>
#include "can_msgs/msg/can_datas.hpp"
#include "can_msgs/msg/can_msg_data.hpp"
#include "can_msgs/msg/can_sig_data.hpp"
#include "can_msgs/msg/frame.hpp"
#include "can_msgs/msg/radar_cam_datas.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zf_global/in/hal_sync_global.h"
#include "zf_global/util/logger.h"
#include "zf_global/util/string_util.h"

// /**
//  * @namespace NS_ZF::driver::synchronization
//  */
BEGIN_NS_ZF_HAL_SYNC

// static const char gk_radarNovaObjNum[] = "Obj_NofObjects";		   //
// 60A static const char gk_radar5G4TLeftB00A[] = "SRR_Left_B0_Obj_0A";   //
// 2566897328 static const char gk_radar5G4TRightC00A[] = "SRR_Right_C0_Obj_0A";
// // 2566897328
// // ORDER C0 C1 B1 B0
// static const char gk_radar5G4TLeftB0[] = "SRR_Left_B0_Obj";
// static const char gk_radar5G4TLeftB1[] = "SRR_Left_B1_Obj";
// static const char gk_radar5G4TRightC0[] = "SRR_Right_C0_Obj";
// static const char gk_radar5G4TRightC1[] = "SRR_Right_C1_Obj";

class RadarPreprocess {
 protected:
  using RadarData = can_msgs::msg::CanDatas;
  std::map<std::string, std::string> __key_dic;
  std::vector<std::string> __key_list;
  std::map<std::string, int> __index_dic;
  std::string m_strData;
  std::string __time_stamp;
  std::unordered_map<int, std::string> __class_def;
  std::unordered_map<int, std::string> __detec_st;
  std::unordered_map<int, std::string> __motion_st;
  std::unordered_map<int, std::string> m_mapStRadar;

 public:
  // virtual std::string ProcessData(const RadarData &data) = 0;

  std::vector<std::string> col_names() {
    std::vector<std::string> col_names;
    col_names.reserve(__key_dic.size());
    for (const auto &entry : __key_dic) {
      col_names.push_back(entry.second);
    }
    return col_names;
  }

  std::unordered_map<int, std::string> ObjClass() { return __class_def; }

  std::unordered_map<int, std::string> DetectionSt() { return __detec_st; }

  std::unordered_map<int, std::string> MotionSt() { return __motion_st; }

  virtual std::string ProcessData(const RadarData &radar) {
    std::stringstream output_stream("");
    output_stream << "time_stamp," << radar.header.stamp.sec << "."
                  << std::setw(9) << std::setfill('0')
                  << radar.header.stamp.nanosec << ",\n";
    output_stream.setf(std::ios::fixed);
    output_stream.precision(4);
    for (const auto &msg : radar.msg_datas) {
      for (const auto &sig : msg.sig_datas) {
        output_stream << sig.sig_name << "," << sig.sig_data << ",";
        // Remove the trailing comma and space
        // std::string output = output_stream.str();
        // output = output.substr(0, output.size() - 2);
        // output_stream.str("");
        // output_stream.clear();
        // output_stream << output << std::endl;
      }
    }
    m_strData = output_stream.str();
    return m_strData;
  }
};

class RadarPreprocessIpmLine {
 public:
  RadarPreprocessIpmLine() {
    m_setLineSigKeyList = {"NumberOfLineObjects",
                           "RoadType",
                           "LaneChangeStatus",
                           "LaneChangeProbability",
                           "C0",
                           "C1",
                           "C2",
                           "C3",
                           "LineClass",
                           "Quality",
                           "ViewRange"};
    m_umapRoadType = {{0, "unknown"}, {1, "highway"}, {2, "inner city"}};
    m_umapLaneChangeStatus = {{0, "No lane change"},
                              {1, "Left lane change"},
                              {2, "Right lane change"}};
    m_umapLineClass = {{0, "undecided"},
                       {1, "solid"},
                       {2, "dashed"},
                       {3, "double line crossable"},
                       {4, "double line uncrossable"},
                       {5, "multiple lines crossable"},
                       {6, "multiple lines uncrossable"},
                       {7, "Botts dots"},
                       {8, "curb"},
                       {9, "snow edge"},
                       {10, "road edge"},
                       {11, "virtual"},
                       {12, "barrier"},
                       {13, "cones"}};
    m_umapLineQuality = {
        {0, "Low1_NoLineDetected"},
        {1, "Low2_InsufficientScore"},
        {2, "Medium_PredictedOrLowScore"},
        {3, "High"},
    };
  }

  void ClearLaneData() { m_dicLineSigs.clear(); }

  std::unordered_map<std::string, std::string> GetLaneData() {
    return m_dicLineSigs;
  }

  void ExtractCanMsg(const can_msgs::msg::CanMsgData &msg) {
    if (!NS_ZF_STRING_UTIL::IsSubStr("Line", msg.msg_name)) {
      return;
    }
    for (const auto &sig : msg.sig_datas) {
      std::string end_name =
          sig.sig_name.substr(sig.sig_name.find_last_of("_") + 1);
      if (!m_setLineSigKeyList.count(end_name)) {
        continue;
      }
      if (end_name == "RoadType") {
        m_dicLineSigs[sig.sig_name] =
            m_umapRoadType[static_cast<int>(sig.sig_data)];
      } else if (end_name == "LaneChangeStatus") {
        m_dicLineSigs[sig.sig_name] =
            m_umapLaneChangeStatus[static_cast<int>(sig.sig_data)];
      } else if (end_name == "LineClass") {
        m_dicLineSigs[sig.sig_name] =
            m_umapLineClass[static_cast<int>(sig.sig_data)];
      } else if (end_name == "Quality") {
        m_dicLineSigs[sig.sig_name] =
            m_umapLineQuality[static_cast<int>(sig.sig_data)];
      } else {
        m_dicLineSigs[sig.sig_name] = std::to_string(sig.sig_data);
      }
    }
  }

 private:
  std::set<std::string> m_setLineSigKeyList;
  std::unordered_map<int, std::string> m_umapRoadType;
  std::unordered_map<int, std::string> m_umapLaneChangeStatus;
  std::unordered_map<int, std::string> m_umapLineClass;
  std::unordered_map<int, std::string> m_umapLineQuality;
  std::unordered_map<std::string, std::string> m_dicLineSigs;
};

class RadarPreprocessIpm : public RadarPreprocess {
 private:
  int __prop_len;
  double __dx;
  double __dy;
  int m_yCoeff;
  std::map<int, std::vector<std::string>> m_dicSigName;
  std::unique_ptr<RadarPreprocessIpmLine> m_ptrProcessLane;

 public:
  RadarPreprocessIpm() : m_ptrProcessLane(new RadarPreprocessIpmLine()) {
    // __key_dic = {{"Identifier", "id"},
    //              {"LongitudinalDistance", "pos_x"},
    //              {"AbsoluteSpeed", "abs_spd"},
    //              {"OrientationAngle", "heading"},
    //              {"ExistenceProbability", "prob"},
    //              {"Class", "class"},
    //              {"DetectionStatus", "status"},
    //              {"MotionStatus", "motion"},
    //              {"LateralDistance", "pos_y"},
    //              {"Width", "width"},
    //              {"Lane", "Lane"},
    //              {"BrakeLight", "BrakeLight"},
    //              {"RelativeVelocity", "rel_spd"},
    //              {"CutInCutOut", "CutInCutOut"},
    //              {"Length", "length"}};
    // __key_list.reserve(__key_dic.size());
    // for (const auto &entry : __key_dic)
    // {
    //     __key_list.push_back(entry.first);
    // }
    __key_list.push_back("Identifier");
    __key_list.push_back("LongitudinalDistance");
    __key_list.push_back("AbsoluteSpeed");
    __key_list.push_back("OrientationAngle");
    __key_list.push_back("ExistenceProbability");
    __key_list.push_back("Class");
    __key_list.push_back("DetectionStatus");
    __key_list.push_back("MotionStatus");
    __key_list.push_back("LateralDistance");
    __key_list.push_back("Width");
    __key_list.push_back("Lane");
    __key_list.push_back("BrakeLight");
    __key_list.push_back("RelativeVelocity");
    __key_list.push_back("CutInCutOut");
    __key_list.push_back("Length");
    __key_list.push_back("Height");
    __prop_len = __key_list.size();
    for (int i = 0; i < __prop_len; ++i) {
      __index_dic[__key_list[i]] = i;
    }
    m_dicSigName = {};
    // for (int i = 0; i < 12; ++i)
    // {
    //     m_dicSigName[i] = std::vector<std::string>(__prop_len, "");
    // }
    __dx = 3.83 + 1.54;
    __dy = 0.0;
    m_yCoeff = -1;
    __time_stamp = "";
    __class_def = {{0, "unknown"},   {1, "truck"},   {2, "car"},
                   {3, "motorbike"}, {4, "bicycle"}, {5, "pedestrian"},
                   {6, "undecided"}};
    __detec_st = {{0, "new"}, {1, "measured"}, {2, "predicted"}};
    __motion_st = {{0, "not defined"},      {1, "stationary"},
                   {2, "passing in"},       {3, "stopped"},
                   {4, "passing out"},      {5, "moving in"},
                   {6, "moving out"},       {7, "preceding"},
                   {8, "moving oncoming"},  {9, "crossing"},
                   {10, "close cut-in"},    {11, "moving unknown"},
                   {12, "stopped unknown"}, {13, "stopped crossing"},
                   {14, "passing unknown"}};
  }

  void set_detaxy(double dx, double dy) {
    __dx = dx;
    __dy = dy;
  }

  std::string ProcessData(const RadarData &radar) {
    m_dicSigName.clear();
    m_ptrProcessLane->ClearLaneData();
    for (const auto &msg : radar.msg_datas) {
      int obj_id = NS_ZF_STRING_UTIL::ExtractNumberFromString(msg.msg_name);
      if (obj_id == -1) {
        m_ptrProcessLane->ExtractCanMsg(msg);
        continue;
      } else {
        if (m_dicSigName.find(obj_id) == m_dicSigName.end()) {
          m_dicSigName[obj_id] = std::vector<std::string>(__prop_len, "");
        }
      }
      for (const auto &sig : msg.sig_datas) {
        std::string end_name =
            sig.sig_name.substr(sig.sig_name.find_last_of("_") + 1);
        auto it = __index_dic.find(end_name);
        if (it != __index_dic.end()) {
          int index = it->second;
          if (end_name == "Class") {
            m_dicSigName[obj_id][index] =
                __class_def[static_cast<int>(sig.sig_data)];
          } else if (end_name == "DetectionStatus") {
            m_dicSigName[obj_id][index] =
                __detec_st[static_cast<int>(sig.sig_data)];
          } else if (end_name == "MotionStatus") {
            m_dicSigName[obj_id][index] =
                __motion_st[static_cast<int>(sig.sig_data)];
          } else if (end_name == "LongitudinalDistance") {
            m_dicSigName[obj_id][index] = std::to_string(sig.sig_data + __dx);
          } else if (end_name == "LateralDistance") {
            m_dicSigName[obj_id][index] =
                std::to_string(m_yCoeff * sig.sig_data + __dy);
          } else {
            m_dicSigName[obj_id][index] = std::to_string(sig.sig_data);
          }
        }
      }
    }
    //__time_stamp = data.header.stamp;
    for (auto it = m_dicSigName.begin(); it != m_dicSigName.end();) {
      if (it->second[0].empty() || std::stoi(it->second[0]) == 0) {
        it = m_dicSigName.erase(it);
      } else {
        ++it;
      }
    }

    std::stringstream output_stream("");
    output_stream << "time_stamp," << radar.header.stamp.sec << "."
                  << std::setw(9) << std::setfill('0')
                  << radar.header.stamp.nanosec << ",";
    for (auto &item : m_dicSigName) {
      output_stream << item.first << ",[";
      const std::vector<std::string> &values = item.second;
      for (const auto &value : values) {
        output_stream << value << ",";
      }
      output_stream << "],";
      // Remove the trailing comma and space
      // std::string output = output_stream.str();
      // output = output.substr(0, output.size() - 2);
      // output_stream.str("");
      // output_stream.clear();
      // output_stream << output << std::endl;
    }
    output_stream << "\n";
    std::unordered_map<std::string, std::string> laneSigs =
        m_ptrProcessLane->GetLaneData();
    for (auto &item : laneSigs) {
      output_stream << item.first << "," << item.second << ",";
    }
    m_strData = output_stream.str();
    return m_strData;
  }
};

class RadarPreprocess5G4T : public RadarPreprocess {
 private:
  int __prop_len;
  std::unordered_map<std::string, std::vector<std::string>> m_dicSigName;
  std::string m_strStRadarB0;
  std::string m_strStRadarC0;

 public:
  RadarPreprocess5G4T() {
    // __key_dic = {
    //     {"ID_A", "id"},
    //     {"rel_Lat_Speed", "rel_spd_lat"},
    //     {"rel_Long_Speed", "rel_spd_lon"},
    //     {"rel_Lat_Pos", "pos_y"},
    //     {"rel_Long_Pos", "pos_x"},
    //     {"TrackingStatus", "status"},
    //     {"Motion_Class", "motion"},
    //     {"Length", "length"},
    //     {"Width", "width"},
    //     {"Lifetime", "lifetime"},
    //     {"ClassConfidence", "class_confidence"},
    //     {"Class", "class"},
    //     {"Quality", "prob"},
    //     {"heading", "heading"},
    //     {"rel_spd", "rel_spd"}};
    // for (auto &kv : __key_dic)
    // {
    //     __key_list.push_back(kv.first);
    // }
    __key_list.push_back("ID_A");
    __key_list.push_back("rel_Lat_Speed");
    __key_list.push_back("rel_Long_Speed");
    __key_list.push_back("rel_Lat_Pos");
    __key_list.push_back("rel_Long_Pos");
    __key_list.push_back("TrackingStatus");
    __key_list.push_back("Motion_Class");
    __key_list.push_back("Length");
    __key_list.push_back("Width");
    __key_list.push_back("Lifetime");
    __key_list.push_back("ClassConfidence");
    __key_list.push_back("Class");
    __key_list.push_back("Quality");
    __key_list.push_back("heading");
    __key_list.push_back("rel_spd");
    __prop_len = __key_list.size();
    for (int i = 0; i < __prop_len; i++) {
      __index_dic[__key_list[i]] = i;
      // std::cout << ", " << __key_list[i].c_str() << ", " << i;
    }
    __class_def = {{0, "unknown"},     {1, "pedestrian"}, {2, "2W"},
                   {3, "car"},         {5, "VRU"},        {6, "non_VRU"},
                   {4, "truck_or_bus"}};
    for (int i = 7; i < 16; i++) {
      __class_def[i] = "not_available";
    }
    __detec_st = {{0, "new"}, {1, "measured"}, {2, "predicted"}};
    __motion_st = {
        {0, "not defined"}, {1, "stationary"}, {2, "moving"}, {3, "stopped"}};
    m_mapStRadar = {{0, "Initializing"},
                    {1, "Fully Operational"},
                    {2, "Performance Limited"},
                    {3, "Temporary Fault - Pending Recovery"},
                    {4, "Permanent Error - Reset to Recover"},
                    {5, "Falling Asleep"},
                    {6, "Shutting Down"},
                    {7, "Power Save / Testbench"}};
  }

  std::string GetTimeStamp() { return __time_stamp; }
  std::unordered_map<std::string, std::vector<std::string>> GetData() {
    return m_dicSigName;
  }

  std::string ProcessData(const RadarData &radar) {
    m_dicSigName.clear();
    m_strStRadarC0 = "";
    m_strStRadarB0 = "";
    for (const auto &msg : radar.msg_datas) {
      if (msg.msg_name == gk_radar5G4TS5C0) {
        for (const auto &sig : msg.sig_datas) {
          // Right_C0_Status Left_B0_Status
          // std::string end_name =
          //     sig.sig_name.substr(sig.sig_name.find_last_of("_") + 1);
          // if (end_name == "Status") {
          //   m_strStRadar = m_mapStRadar[static_cast<int>(sig.sig_data)];
          //   break;
          // }
          if (sig.sig_name == "Right_C0_Status") {
            m_strStRadarC0 = m_mapStRadar[static_cast<int>(sig.sig_data)];
            break;
          }
        }
        continue;
      }
      if (msg.msg_name == gk_radar5G4TS1B0) {
        for (const auto &sig : msg.sig_datas) {
          // Left_B0_Status
          if (sig.sig_name == "Left_B0_Status") {
            m_strStRadarB0 = m_mapStRadar[static_cast<int>(sig.sig_data)];
            break;
          }
        }
        continue;
      }
      std::string obj_id = msg.msg_name.substr(0, msg.msg_name.size() - 1);
      if (m_dicSigName.find(obj_id) == m_dicSigName.end()) {
        m_dicSigName[obj_id] = std::vector<std::string>(__prop_len, "");
      }
      for (const auto &sig : msg.sig_datas) {
        std::string t_name =
            NS_ZF_STRING_UTIL::RemoveNumberFromString(sig.sig_name);
        std::string end_name = "";
        size_t found = t_name.find("Obj_");
        if (found != std::string::npos) {
          end_name = t_name.substr(found + 4);
          if (!end_name.compare(end_name.size() - 1, end_name.size(), "_")) {
            end_name = end_name.substr(0, end_name.size() - 1);
          }
        }
        if (__index_dic.find(end_name) == __index_dic.end()) {
          continue;
        }
        int index = __index_dic[end_name];
        // LOG_DEBUG() << "---" << index << " and " << end_name.c_str() << ", "
        // << sig.sig_name.c_str();
        if (index >= 0) {
          if (end_name == "Class") {
            m_dicSigName[obj_id][index] =
                __class_def[static_cast<int>(sig.sig_data)];
          } else if (end_name == "TrackingStatus") {
            m_dicSigName[obj_id][index] =
                __detec_st[static_cast<int>(sig.sig_data)];
          } else if (end_name == "Motion_Class") {
            m_dicSigName[obj_id][index] =
                __motion_st[static_cast<int>(sig.sig_data)];
          } else if (end_name == "rel_Long_Pos") {
            m_dicSigName[obj_id][index] = std::to_string(sig.sig_data + 3.83);
          } else {
            m_dicSigName[obj_id][index] = std::to_string(sig.sig_data);
          }
        }
      }
    }

    for (auto it = m_dicSigName.begin(); it != m_dicSigName.end();) {
      if (it->second[0].empty()) {
        LOG_WARN() << "5G4T obj incomplete: " << it->first.c_str() << ", "
                   << "time_stamp," << radar.header.stamp.sec << "."
                   << std::setw(9) << std::setfill('0')
                   << radar.header.stamp.nanosec;
        it = m_dicSigName.erase(it);
      }
      if (std::stoi(it->second[0]) == 0) {
        it = m_dicSigName.erase(it);
      } else {
        ++it;
      }
    }

    for (auto &item : m_dicSigName) {
      std::vector<std::string> &value = item.second;
      if (value[1].empty() || value[2].empty()) {
        // std::cout << "--------------------None" << std::endl;
        // std::cout << m_dicSigName << std::endl;
        continue;
      }
      double rel_spd_lat = std::stod(value[1]);
      double rel_spd_lon = std::stod(value[2]);
      value[__prop_len - 2] =
          std::to_string(std::atan2(rel_spd_lat, rel_spd_lon));
      value[__prop_len - 1] = std::to_string(
          std::sqrt(rel_spd_lat * rel_spd_lat + rel_spd_lon * rel_spd_lon));
    }

    //__time_stamp = data.header.stamp;

    std::stringstream output_stream("");
    output_stream << "time_stamp," << radar.header.stamp.sec << "."
                  << std::setw(9) << std::setfill('0')
                  << radar.header.stamp.nanosec << ",";
    for (auto &item : m_dicSigName) {
      output_stream << item.first << ",[";
      const std::vector<std::string> &values = item.second;
      for (const auto &value : values) {
        output_stream << value << ",";
      }
      output_stream << "],";
      // Remove the trailing comma and space
      // std::string output = output_stream.str();
      // output = output.substr(0, output.size() - 2);
      // output_stream.str("");
      // output_stream.clear();
      // output_stream << output << std::endl;
    }
    output_stream << "\nstatus," << m_strStRadarC0 << "," << m_strStRadarB0;
    m_strData = output_stream.str();
    return m_strData;
  }
};

END_NS_ZF_HAL_SYNC

#endif /* ZF_HAL_RADAR_PREPROCESS_H_ */
