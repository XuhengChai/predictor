#include "zf_hal_canmsg_parser/hal_canmsg_pp_node.h"

#include "rclcpp_components/register_node_macro.hpp"
#include "zf_global/util/logger.h"
#include "zf_global/util/string_util.h"
#include "zf_global/zf_global_topic_name.h"
#include "zf_hal_can_driver/byte.h"
#include "zf_hal_can_driver/client/can_frame.h"
#include "zf_hal_can_driver/hal_can_id.h"
#include "zf_hal_canmsg_parser/hal_can_pgn_define.h"

BEGIN_NS_ZF_DRIVER_CANBUS

using namespace can_msgs;
using namespace std::chrono_literals;
// LOGGER::FileLogger LOG_FILE_PARSER("build_at_" __DATE__ "_" __TIME__
// "parser.log");
LOGGER::FileLogger LOG_FILE_PARSER;
static const char gk_sigNameTrailerPresentState[] =
    "TrailerPresentState"; // msg name PropB_Trailer
static const char gk_sigNameDrvrBrk[] =
    "DrvrActvtnDmndFrAdvncdEmrgncyBrk";            // msg name AEBS2_27 FOR AC1000T
static const char gk_dbcNameVehicle[] = "Vehicle"; // VehicleCAN.dbc
static const int gk_iCanIdTSC1 = 0xC000080;
static const int gk_iCanIdXBR = 0xC040B80;

CanMsgParserPackNode::CanMsgParserPackNode(const rclcpp::NodeOptions &options,
                                           std::string name)
    : Node(name, options)
{
  m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::INIT);
  std::cout << "-------------------------------Hello i am "
               "CanMsgParserPackNode----------------- \n";
  m_vDbcFilesPath =
      this->declare_parameter<StringVector>("dbc_files_path", StringVector({}));
  m_strInterface = this->declare_parameter("interface", "can0");
  m_strVehicleInterface =
      this->declare_parameter("vehicle_can_interface", "can0");
  m_iCanDescription = this->declare_parameter("can_description", 0);
  if (m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_VEHICLE)
  {
    InitVehicleVars();
  }
  m_iB0Status = -1; // init with no meaning sicne Status < 7
  m_iC0Status = -1;

  RCLCPP_INFO(this->get_logger(), "m_strInterface: %s", m_strInterface.c_str());
  RCLCPP_INFO(this->get_logger(), "m_iCanDescription: %d", m_iCanDescription);
  m_bEnableDebug = this->declare_parameter<bool>("enable_debug", false);
  m_iLogCnt = this->declare_parameter("log_cnt", 0);
  // auto fileName = "build_" + std::to_string(m_iLogCnt) + "_at_" __DATE__ "_"
  // __TIME__ "parser.log";
  auto fileName =
      "build_" + std::to_string(m_iLogCnt) + "_at_" __DATE__ "_parser.log";
  LOG_FILE_PARSER.SetFileName(fileName);

  for (const auto &dbcPath : m_vDbcFilesPath)
  {
    auto dbcName = CanMsgParser::Instance().AddDBCParser(dbcPath);
    m_vDbcName.push_back(dbcName);
    LOG_INFO() << "dbcPath is" << dbcPath.c_str();
    FillInCanData(dbcName);
    // m_mapDbcNamePath.insert(std::make_pair(dbcName, dbcPath));
    // CanMsgParser::Instance().PrintDBCParser(dbcName);
  }

  rclcpp::QoS video_qos(500);
  video_qos.keep_last(500);
  // video_qos.reliable();
  video_qos.best_effort(); // EngSpeedAtPoint2
  video_qos.durability_volatile();
  std::string topicName =
      "/" + m_strInterface + gk_halFromCanRawBus; // from_raw_can_bus
  m_subRawCanData = this->create_subscription<msg::Frame>(
      topicName, video_qos,
      std::bind(&CanMsgParserPackNode::ParseRawCanDataCallback, this,
                std::placeholders::_1));
  // if (!m_isVehicleCan)
  if (m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_AC1000T ||
      m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_5G4T)
  {
    InitSet();
    topicName = "/" + m_strVehicleInterface +
                gk_halFromCanRawBus; // /can0/hal/from_can_raw_bus
    m_subVehicleCanData = this->create_subscription<msg::Frame>(
        topicName, video_qos,
        std::bind(&CanMsgParserPackNode::VehicleRawCanDataCallback, this,
                  std::placeholders::_1));
  }
  topicName = "/" + m_strInterface + gk_halFromCanMsgData; // from_can_msg_data
  m_pubMsgData = this->create_publisher<msg::CanMsgData>(topicName, video_qos);

  topicName = "/" + m_strInterface + gk_halToCanMsgData; // to_can_msg_data
  m_subMsgData = this->create_subscription<msg::CanMsgData>(
      topicName, video_qos,
      std::bind(&CanMsgParserPackNode::PackMsgDataCallback, this,
                std::placeholders::_1));
  topicName = "/" + m_strInterface + gk_halToCanSigData; // to_can_sig_data
  m_subSigData = this->create_subscription<msg::CanSigData>(
      topicName, video_qos,
      std::bind(&CanMsgParserPackNode::PackSigDataCallback, this,
                std::placeholders::_1));
  topicName = "/" + m_strInterface + gk_halToCanRawBus; // to_raw_can_bus
  m_pubRawCanData = this->create_publisher<msg::Frame>(topicName, video_qos);
  topicName = "/" + m_strInterface + gk_halToUi;
  m_pubStwInfo = this->create_publisher<std_msgs::msg::String>(topicName, video_qos);
  topicName = "/" + m_strInterface + gk_halRadarStatusToUi;
  m_pubStrRadarStatus = this->create_publisher<std_msgs::msg::String>(topicName, video_qos);
  if (m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_AC1000T ||
      m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_5G4T)
  // if (
  //     m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_5G4T)
  {
    InitTimerTransmit();
  }

  rclcpp::QoS status_qos(50);
  status_qos.keep_last(50);
  status_qos.reliable();
  status_qos.durability_volatile();
  topicName =
      "/" + m_strInterface + gk_halNodeStatusCanMsg; // /node_status/can_msg
  m_pubNodeStatus =
      this->create_publisher<std_msgs::msg::UInt32>(topicName, status_qos);
  m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::INIT);
  // LOG_DEBUG() << "m_statusMsg PP " << m_statusMsg.data;
  m_timerNodeSataus = create_wall_timer(
      100ms, [this]()
      { m_pubNodeStatus->publish(m_statusMsg); 
      std::ostringstream oss;
      oss << m_iB0Status << " " << m_iC0Status;
      m_msgStrRadarStatus.data = oss.str();
      m_pubStrRadarStatus->publish(m_msgStrRadarStatus); });
}

CanMsgParserPackNode::~CanMsgParserPackNode() {}

// Function taking in DBC_out metadata vector and clones its metadata to
// message_data
void CanMsgParserPackNode::FillInCanData(const std::string &name)
{
  auto dbcIt = CanMsgParser::Instance().GetDBCParser(name);

  rclcpp::Subscription<msg::CanMsgData>::SharedPtr m_subMsgData;
  rclcpp::Subscription<msg::CanSigData>::SharedPtr m_subSigData;

  for (const auto &message : *dbcIt)
  {
    msg::CanMsgData msgData;
    msgData.msg_name = message.second.getName();
    msgData.msg_pgn = message.second.getPGN();
    msgData.msg_id = message.second.getId();
    // std::cout << message.second.getName() << " " << std::hex << std::showbase
    // << message.second.getId() << std::endl; std::cout <<
    // message.second.getName() << " 0x" << message.second.getPGN() <<
    // std::endl; Fill Message Data Object
    for (const auto &sig : message.second)
    {
      msg::CanSigData sigData;
      sigData.sig_name = sig.second.getName();
      sigData.sig_unit = sig.second.getUnit();
      sigData.sig_data = 0.0;
      sigData.is_valid = false;
      msgData.sig_datas.push_back(sigData);
      // std::cout << sig.second.getName() << " ---" <<
      // sig.second.getLsb().value << " ---" << sig.second.getMsb().value <<
      // std::endl;
    }
    if (m_pCanDatas.find(msgData.msg_pgn) == m_pCanDatas.end())
    {
      m_pCanDatas[msgData.msg_pgn] = std::make_shared<msg::CanDatas>();
    }
    auto &vec = m_pCanDatas[msgData.msg_pgn]->msg_datas;
    if ((msgData.msg_id & 0xFFFF) != 0xFEFE)
    {
      vec.push_back(msgData);
    }
    else
    {
      vec.insert(vec.begin(), msgData);
    }
  }
}

// Function which returns index of metadata array, given msg_id.
int CanMsgParserPackNode::Index(const uint32_t &msg_id)
{
  auto pgn = Byte::PGNFromCanId(msg_id);
  // Inputs message and outputs its "index" in m_pCanDatas
  if (m_pCanDatas.find(pgn) == m_pCanDatas.end())
  {
    return -1; // If message id not found
  }
  auto vecs = m_pCanDatas[pgn]->msg_datas;
  for (int i = 0; i < vecs.size(); i++)
  {
    if (vecs.at(i).msg_id == msg_id)
    {
      return i;
    }
  }
  for (int i = 0; i < vecs.size(); i++)
  {
    if (Byte::EcuFromCanId(vecs.at(i).msg_id) == Byte::EcuFromCanId(msg_id))
    {
      return i;
    }
  }
  return 0;
}

void CanMsgParserPackNode::ParseRawCanDataCallback(
    const msg::Frame::ConstSharedPtr raw)
{
  bool skipPub = false;
  bool isC0 = false;
  bool isB0 = false;
  switch (m_iCanDescription)
  {
  case CANCardParameter::CANDescription::CAN_DES_AC1000T:
    if (m_mapTransferIds.find(raw->id) == m_mapTransferIds.end() || m_setTransferIds.count(raw->id))
      return;
  case CANCardParameter::CANDescription::CAN_DES_5G4T:
  {
    if (m_setTransferIds.count(raw->id))
      return;
    skipPub = (((raw->id >> 12) & 0xF) == 6); // skip PGN is FF6x...
    if (raw->id == ECanId::SRR_S5_C0)
    {
      isC0 = true;
    }
    else if (raw->id == ECanId::SRR_S1_B0)
    {
      isB0 = true;
    };
    if (isB0 || isC0)
    {
      msg::CanMsgData msgData;
      CanMsgParser::SignalPPMap parserSigMap;
      auto errCode = Raw2MsgData(raw, msgData, parserSigMap);
      if (errCode != ErrorCode::OK)
      {
        m_statusMsg.data = static_cast<uint32_t>(errCode);
        if (m_bEnableDebug)
        {
          LOG_FILE_PARSER() << "Parser Data Error";
        }
        return;
      }
      m_pubMsgData->publish(msgData);
      if (isB0)
      {
        m_iB0Status = parserSigMap.at("Left_B0_Status");
        return;
      }
      if (isC0)
      {
        m_iC0Status = parserSigMap.at("Right_C0_Status");
        return;
      }
    }
    break;
  }
  default:
    break;
  }
  // if (m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_AC1000T ||
  //     m_iCanDescription ==
  //         CANCardParameter::CANDescription::
  //             CAN_DES_5G4T) // since AC1000T is connected to vehicle can.
  // {
  //   if (m_setTransferIds.count(raw->id)) // filter can id data sent by itself
  //   {
  //     return;
  //   }
  //   // VehicleRawCanDataCallback(raw);
  // }
  // // std_msgs::msg::Header head;
  // // head.stamp = this->now();
  // // LOG_FILE_PARSER() << "[" << raw->header.stamp.sec << "." << std::setw(9) <<
  // // std::setfill('0') << raw->header.stamp.nanosec
  // //                << "]-" << "[" << head.stamp.sec << "." << std::setw(9) <<
  // //                std::setfill('0') << head.stamp.nanosec
  // //                << "]-" << raw->id;
  // if (m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_AC1000T &&
  //     m_mapTransferIds.find(raw->id) == m_mapTransferIds.end())
  // {
  //   return;
  // }
  msg::CanMsgData msgData;
  CanMsgParser::SignalPPMap parserSigMap;
  auto errCode = Raw2MsgData(raw, msgData, parserSigMap);
  if (errCode != ErrorCode::OK)
  {
    // LOG_ERROR() << "Parser Data Error " << uint32_t(errCode) << "  "  << raw->id;
    if (m_iCanDescription != CANCardParameter::CANDescription::CAN_DES_IPM ||
        errCode != ErrorCode::CAN_PARSE_INVALID_NAME)
    {
      m_statusMsg.data = static_cast<uint32_t>(errCode);
    }

    if (m_bEnableDebug)
    {
      LOG_FILE_PARSER() << "Parser Data Error";
    }
    return;
  }
  // LOG_INFO() << "success " << msgData.msg_name << ", " << msgData.msg_pgn
  //            << ", " << msgData.sig_datas[0].sig_name
  //            << ", " << msgData.sig_datas[0].sig_data;
  if (m_bEnableDebug)
  {
    LOG_FILE_PARSER() << LogString(msgData);
  }
  if (skipPub)
    return;
  m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::READY);
  if (raw->id == ECanId::StwError2Orin) // for StwError2Orin 18FF3181
  {
    // LOG_INFO() << "success " << msgData.msg_name << ", " << msgData.msg_pgn
    //            << ", " << msgData.sig_datas[0].sig_name
    //            << ", " << msgData.sig_datas[0].sig_data;
    auto message = std_msgs::msg::String();
    std::ostringstream oss;
    for (auto &sig : msgData.sig_datas)
    {
      oss << sig.sig_data;
    }
    message.data = oss.str();
    m_pubStwInfo->publish(message);
    return;
  }
  if (!m_isVehicleCan)
  {
    m_pubMsgData->publish(msgData);
    return;
  }
  bool isInTimeout = m_indexMsgTimeout.find(raw->id) != m_indexMsgTimeout.end();
  if (isInTimeout || (m_mapSigsVehicle.find(raw->id) != m_mapSigsVehicle.end()))
  {
    if (isInTimeout)
    {
      m_indexMsgTimeout[raw->id][2] = 0;
    }
    std::vector<SignalAlias> tEgoSignals = m_mapSigsVehicle[raw->id];
    std::lock_guard<std::mutex> guard(m_lockSigs2Ego);
    for (auto &egoSig : tEgoSignals)
    {
      m_mapSigs2Ego[egoSig.alias] = parserSigMap.at(egoSig.sigName);
    }
  }
}

void CanMsgParserPackNode::VehicleRawCanDataCallback(
    const can_msgs::msg::Frame::ConstSharedPtr raw)
{
  auto pgn = Byte::PGNFromCanId(raw->id);
  auto srcAddr = Byte::EcuFromCanId(raw->id);
  // if ((!pgn.compare(gk_pgnEBC2)) ||
  //     !pgn.compare(gk_pgnETC2) ||
  //     !pgn.compare(gk_pgnVDC2))
  // {
  //   m_pubRawCanData->publish(*raw);
  // }
  if (!m_mapPgnSet[m_iCanDescription].count(pgn))
  {
    return;
  }

  can_msgs::msg::Frame data = *raw;
  bool modified = false;
  for (const auto &dbcName : m_vDbcName)
  {
    if (NS_ZF_STRING_UTIL::IsSubStr(gk_dbcNameVehicle, dbcName))
    {
      continue;
    }
    if (!CanMsgParser::Instance().CheckValid(dbcName, pgn))
    {
      continue;
    }
    auto dbcIt = CanMsgParser::Instance().GetDBCParser(dbcName);
    // auto oldId = data.id;
    // LOG_DEBUG() << "Old Id: "
    //               << "[" << Byte::Int2Hex(raw->id).c_str() << "]" <<
    //               pgn.c_str() << ", " << srcAddr.c_str();
    if (dbcIt->ModifyId(data.id, pgn, srcAddr))
    {
      m_setTransferIds.insert(data.id);
      // LOG_DEBUG() << "Old, ModifyId: "
      //             << "[" << Byte::Int2Hex(raw->id).c_str() << ", " <<
      //             Byte::Int2Hex(data.id).c_str() << "]";
      modified = true;
      break;
    }
  }

  if (m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_5G4T)
  {
    modified = ModifyDataTo5G4T(data, pgn);
  }
  // if (!modified)
  // {
  //   return;
  // }
  // LOG_INFO() << data.is_extended << "publish: "
  //            << "[" << Byte::Int2Hex(data.id).c_str() << "]";
  // TODO check extend id

  m_pubRawCanData->publish(data);
}

void CanMsgParserPackNode::PackMsgDataCallback(
    const msg::CanMsgData::ConstSharedPtr msg)
{
  if (msg->msg_id == gk_iCanIdTSC1)
  {
    std::lock_guard<std::mutex> guard(m_lockTSC1);
    m_canMsgDataTSC1.sig_datas.clear();
    m_canMsgDataTSC1.header.stamp = this->now();
    for (auto &sig : msg->sig_datas)
    {
      auto sig_data = msg::CanSigData();
      sig_data.sig_name = sig.sig_name;
      sig_data.sig_data = sig.sig_data;
      m_canMsgDataTSC1.sig_datas.push_back(sig_data);
      /* code */
    }
    return;
  }
  if (msg->msg_id == gk_iCanIdXBR)
  {
    std::lock_guard<std::mutex> guard(m_lockXBR);
    m_canMsgDataXBR.sig_datas.clear();
    m_canMsgDataXBR.header.stamp = this->now();
    for (auto &sig : msg->sig_datas)
    {
      auto sig_data = msg::CanSigData();
      sig_data.sig_name = sig.sig_name;
      sig_data.sig_data = sig.sig_data;
      m_canMsgDataXBR.sig_datas.push_back(sig_data);
      /* code */
    }
    return;
  }
  msg::Frame rawData;
  auto errCode = Msg2RawData(msg, rawData);
  if (errCode != ErrorCode::OK)
  {
    LOG_ERROR() << "PackMsgData Error, name: " << msg->msg_name.c_str()
                << "id: " << msg->msg_id
                << "pgn: " << Byte::PGNFromCanId(msg->msg_id).c_str();
    m_statusMsg.data = static_cast<uint32_t>(errCode);
    return;
  }
  // m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::READY);
  m_pubRawCanData->publish(rawData);
}

void CanMsgParserPackNode::PackSigDataCallback(
    const msg::CanSigData::ConstSharedPtr sig)
{
  // msg::Frame rawData;
  // if (Sig2RawData(msg, rawData) != ErrorCode::OK)
  // {
  //   LOG_ERROR() << "Pack Sig Data Error, name: " << sig->sig_name.c_str() <<
  //   "value: " << sig->sig_data; return;
  // }
  // m_pubRawCanData->publish(rawData);
}

ErrorCode CanMsgParserPackNode::Raw2MsgData(const msg::Frame::ConstSharedPtr m,
                                            msg::CanMsgData &msgDatas, CanMsgParser::SignalPPMap &tSigMap)
{
  auto pgn = Byte::PGNFromCanId(m->id);
  int index =
      Index(m->id); // Corresponding to index in DBC_out "metadata" vector
  // Check incase message id not found
  if (index == -1)
  {
    // RCLCPP_ERROR(this->get_logger(), "sd_state_msgs.cpp: Unknown Message ID: %u", m->id);
    return ErrorCode::CAN_PARSE_INVALID_NAME;
  }
  std::string excepText = "Cannot find DBCParser with ";
  bool validName = false;
  for (const auto &dbcName : m_vDbcName)
  {
    if (!CanMsgParser::Instance().CheckValid(dbcName, pgn))
    {
      excepText += "dbc name: " + dbcName + ", pgn: " + pgn + " ";
      continue;
    }
    validName = true;
    CanDataInfo info(m->id, m->data, dbcName);
    info.SetTimeStamp(m->header.stamp.sec, m->header.stamp.nanosec);

    auto errorCode = CanMsgParser::Instance().Parse(tSigMap, info);
    if (errorCode != ErrorCode::OK)
    {
      return errorCode;
    }
    else
    {
      if (info.GetPGNType() == CanDataInfo::PGNType::TPDT)
      {
        auto iPGN = static_cast<uint32_t>(tSigMap[gk_keyTPDTpgn]);
        index = Index(iPGN);
        // // tSigMap[gk_keyTPDTpgn] = 65251, //EB00, val: 65251, FEE3, 0
        // 418119424, EB00, val:, 419357440, FEE3, 0
        // LOG_INFO() << m->id << ", " << pgn.c_str() << ", val:"
        //            << ", " << iPGN << ", "<< Byte::PGNFromCanId(iPGN) << ", "
        //            << index;
        msgDatas = m_pCanDatas[Byte::PGNFromCanId(iPGN)]->msg_datas[index];
        // LOG_INFO() << LogString(msgDatas).c_str();
      }
      else
      {
        // m_pCanDatas[pgn]->msg_datas[index].msg_id = m->id;
        msgDatas = m_pCanDatas[pgn]->msg_datas[index];
        msgDatas.msg_id = m->id;
      }
      for (auto &sig : msgDatas.sig_datas)
      {
        sig.is_valid = true;
        sig.sig_data = tSigMap[sig.sig_name];
      }
    }

    // for (auto &sig : datas)
    // {
    //   LOG_INFO() << sig.sig_name.c_str() << " and " << sig.sig_data;
    // }
    break;
  }
  msgDatas.header = m->header;
  if (!validName)
  {
    LOG_ERROR() << excepText.c_str();
    return ErrorCode::CAN_PARSE_INVALID_NAME;
  }
  // msgDatas = m_pCanDatas[pgn]->msg_datas[index];

  return ErrorCode::OK;
}

NS_ZF::ErrorCode CanMsgParserPackNode::Msg2RawData(
    const can_msgs::msg::CanMsgData::ConstSharedPtr cData,
    can_msgs::msg::Frame &raw)
{
  // Pack(const SignalPackVector &signals, const std::string &dbcName, uint32_t
  // address)
  auto msgData = *cData;
  if (!msgData.has_msg_id)
  {
    bool tValidName = false;

    for (const auto &dbcName : m_vDbcName)
    {
      auto dbcIt = CanMsgParser::Instance().GetDBCParser(dbcName);
      if (dbcIt->CheckName(msgData.msg_name))
      {
        tValidName = true;
        auto msg = dbcIt->getMsgByName(msgData.msg_name);
        msgData.msg_id = CanId::GenerateCanId(3, msg.getPGN(), msgData.src_id,
                                              msgData.des_id);
      }
    }
    if (!tValidName)
    {
      return ErrorCode::CAN_PARSE_INVALID_NAME;
    }
  }

  auto pgn = Byte::PGNFromCanId(msgData.msg_id);
  msgData.msg_pgn = pgn;
  int index = Index(
      msgData.msg_id); // Corresponding to index in DBC_out "metadata" vector
  // Check incase message id not found
  if (index == -1)
  {
    RCLCPP_WARN(this->get_logger(), "sd_state_msgs.cpp: Unknown Message ID: %u",
                msgData.msg_id);
    return ErrorCode::CAN_PARSE_INVALID_NAME;
  }
  ErrorCode errCode;
  std::string excepText = "Cannot find DBCParser with ";
  bool validName = false;
  for (const auto &dbcName : m_vDbcName)
  {
    if (!CanMsgParser::Instance().CheckValid(dbcName, pgn))
    {
      excepText += "dbc name: " + dbcName + ", pgn: " + pgn + " ";
      continue;
    }
    auto dbcIt = CanMsgParser::Instance().GetDBCParser(dbcName);
    CanMsgParser::SignalPPMap tSigMap;
    for (auto &sig : msgData.sig_datas)
    {
      sig.sig_unit = dbcIt->at(pgn).at(sig.sig_name).getUnit();
      sig.is_valid = true;
      // SignalPackValue tv{sig.getName(), GetValue<CanDataType>(dat, sig)};
      tSigMap.insert(std::make_pair(sig.sig_name, sig.sig_data));
      // LOG_INFO() << sig.sig_name.c_str() << " and " << sig.sig_data;
    }
    {
      // std::lock_guard<std::mutex> lock(m_mtx);
      msgData.msg_name = m_pCanDatas[pgn]->msg_datas[index].msg_name;
      // m_pCanDatas[pgn]->msg_datas[index] = msgData;
    }
    validName = true;
    errCode = CanMsgParser::Instance().Pack(raw.data, tSigMap, dbcName,
                                            msgData.msg_id);
    break;
  }
  if (!validName)
  {
    LOG_ERROR() << excepText.c_str();
    errCode = ErrorCode::CAN_PARSE_INVALID_NAME;
  }

  CanId receive_id(msgData.msg_id, FrameType::DATA, msgData.is_extended);
  raw.id = receive_id.Id();
  raw.dlc = static_cast<uint8_t>(raw.data.size());
  raw.is_error = (receive_id.GetFrameType() == FrameType::ERROR);
  raw.is_rtr = (receive_id.GetFrameType() == FrameType::REMOTE);
  raw.is_extended = receive_id.IsExtended();
  raw.header = msgData.header;

  // std::stringstream output_stream("");
  // output_stream << "[" << msgData.msg_id
  //               << "], id: " << pgn
  //               // << ", " << Byte::Int2Hex(IdentifierID())
  //               << ", " << raw.id
  //               << ", data:";
  // for (uint8_t i = 0; i < raw.dlc; ++i)
  // {
  //   output_stream << Byte::Int2Hex(raw.data[i]) << " ";
  // }
  // output_stream << ",";
  // LOG_INFO() << output_stream.str().c_str();

  if (errCode != ErrorCode::OK)
  {
    return errCode;
  }
  return ErrorCode::OK;
}

NS_ZF::ErrorCode CanMsgParserPackNode::Sig2RawData(
    const can_msgs::msg::CanSigData::ConstSharedPtr sigData,
    can_msgs::msg::Frame &raw)
{
  return ErrorCode::OK;
}

void CanMsgParserPackNode::InitTimerTransmit()
{
  // TODO test in vehicle
  m_canDataOWW.id = 0x18FDCD21;
  // m_canDataOWW.id = 0x18FDCDF0;
  std::vector<uint8_t> dataOWW{0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  m_canDataOWW.data.swap(dataOWW);
  m_canDataOWW.dlc = 8;
  m_canDataOWW.is_extended = true;
  can_msgs::msg::CanMsgData msgData;
  m_canMsgDataAEBS2_27.msg_id = 0x0C0BA027;
  m_canMsgDataAEBS2_27.has_msg_id = true;
  m_canMsgDataAEBS2_27.is_extended = true;
  can_msgs::msg::CanSigData sigData;
  sigData.sig_name = gk_sigNameDrvrBrk;
  sigData.sig_data = 1.0;
  m_canMsgDataAEBS2_27.sig_datas.emplace_back(sigData);

  if (m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_VEHICLE)
  {
    m_canMsgDataTSC1.msg_id = gk_iCanIdTSC1;
    m_canMsgDataTSC1.has_msg_id = true;
    m_canMsgDataTSC1.is_extended = true;
    m_canMsgDataTSC1.header.stamp = this->now();
    m_timerTSC1 = create_wall_timer(10ms, [this]()
                                    { PubTSC1();
                                      PubVehicleCanMsg2Ego(); });

    m_canMsgDataXBR.msg_id = gk_iCanIdXBR;
    m_canMsgDataXBR.has_msg_id = true;
    m_canMsgDataXBR.is_extended = true;
    m_canMsgDataXBR.header.stamp = this->now();
    m_timerXBR = create_wall_timer(20ms, [this]()
                                   { PubXBR(); });
  }

  // m_canDataVDHR.id = 0x18FEC1EE;

  // m_canDataTD_EE.id = 0x18FEE6EE;
  // std::vector<uint8_t>(8, 0X0).swap(m_canDataTD_EE.data);
  // m_canDataTD_EE.dlc = 8;
  // m_canDataTD_EE.is_extended = true;
  if (m_iCanDescription == CANCardParameter::CANDescription::CAN_DES_AC1000T)
  {
    LOG_DEBUG() << "m_iCanDescription CAN_DES_AC1000T" << m_iCanDescription;
    m_canDataOWW.id = 0x18FDCD32;
    m_canDataERC1_29_AC1000T.id = 0x18F00029;
    std::vector<uint8_t>(8, 0X0).swap(m_canDataERC1_29_AC1000T.data);
    m_canDataERC1_29_AC1000T.dlc = 8;
    m_canDataERC1_29_AC1000T.is_extended = true;
    m_timerERC1_AC1000T = create_wall_timer(100ms, [this]()
                                            { m_pubRawCanData->publish(m_canDataERC1_29_AC1000T); });

    m_canDataCCSS_AC1000T.id = 0x18FEED00;
    std::vector<uint8_t>(8, 0X0).swap(m_canDataCCSS_AC1000T.data);
    m_canDataCCSS_AC1000T.dlc = 8;
    m_canDataCCSS_AC1000T.is_extended = true;
    m_timerCCSS_AC1000T = create_wall_timer(
        100ms, [this]()
        { m_pubRawCanData->publish(m_canDataCCSS_AC1000T); });

    // m_canDataFLIC_E8_27_AC1000T.id = 0x18A9E821;
    m_canDataFLIC_E8_27_AC1000T.id = 0x18A9E827;
    std::vector<uint8_t> dataFLIC_E8(8, 0X0);
    dataFLIC_E8[0] = 0X01;
    dataFLIC_E8.swap(m_canDataFLIC_E8_27_AC1000T.data);
    m_canDataFLIC_E8_27_AC1000T.dlc = 8;
    m_canDataFLIC_E8_27_AC1000T.is_extended = true;
    m_timerFLIC_E8_27_AC1000T = create_wall_timer(50ms, [this]()
                                                  {
      m_pubRawCanData->publish(m_canDataFLIC_E8_27_AC1000T);
      PubAEBS2_27ForAC1000T(); });
  }
  else if (m_iCanDescription ==
           CANCardParameter::CANDescription::CAN_DES_5G4T)
  {
    // m_canDataVDHR.id = 0x18FEC117;
    // std::vector<uint8_t> dataVDHR(8, 0X0);
    // m_canDataVDHR.data.swap(dataVDHR);
    // m_canDataVDHR.dlc = 8;
    // m_canDataVDHR.is_extended = true;

    // TrailerPresentState 3 "Not available" 2 "Error" 1 "Trailer present" 0
    // "Trailer not present" ;
    can_msgs::msg::CanMsgData msgData;
    msgData.msg_id = 0x18FF5EF0;
    msgData.has_msg_id = true;
    msgData.is_extended = true;
    can_msgs::msg::CanSigData sigData;
    sigData.sig_name = gk_sigNameTrailerPresentState;
    sigData.sig_data = 1.0;
    msgData.sig_datas.emplace_back(sigData);
    msgData.header.stamp = this->now();

    const msg::CanMsgData::ConstSharedPtr msg =
        std::make_shared<const can_msgs::msg::CanMsgData>(msgData);
    if (Msg2RawData(msg, m_canDataTrailer5G4T) != ErrorCode::OK)
    {
      LOG_ERROR() << "Pack 5G4T MsgData Error, name: " << msg->msg_name.c_str()
                  << "id: " << msg->msg_id
                  << "pgn: " << Byte::PGNFromCanId(msg->msg_id).c_str();
      return;
    }
    m_timerTrailer5G4T = create_wall_timer(
        100ms, [this]()
        { m_pubRawCanData->publish(m_canDataTrailer5G4T); });
  }
  m_timerOWW = create_wall_timer(
      200ms, [this]()
      { m_pubRawCanData->publish(m_canDataOWW); });
  // m_timerVDHR_TDEE = create_wall_timer(1000ms, [this]()
  //                                        {
  //                                        m_pubRawCanData->publish(m_canDataVDHR);
  //                                          m_pubRawCanData->publish(m_canDataTD_EE);
  //                                          });
  // m_timerTDEE = create_wall_timer(1000ms, [this]()
  //                                 {
  //                                 m_pubRawCanData->publish(m_canDataTD_EE);});
}

bool CanMsgParserPackNode::PubXBR()
{
  const msg::CanMsgData::ConstSharedPtr msg =
      std::make_shared<const can_msgs::msg::CanMsgData>(m_canMsgDataXBR);
  {
    std::lock_guard<std::mutex> guard(m_lockXBR);
    if (Msg2RawData(msg, m_canDataXBR) != ErrorCode::OK)
    {
      LOG_ERROR() << "Pack m_canDataXBR Error, name: " << msg->msg_name.c_str()
                  << "id: " << msg->msg_id
                  << "pgn: " << Byte::PGNFromCanId(msg->msg_id).c_str();
      return false;
    }
  }
  m_pubRawCanData->publish(m_canDataXBR);
  return true;
}

bool CanMsgParserPackNode::PubTSC1()
{
  const msg::CanMsgData::ConstSharedPtr msg =
      std::make_shared<const can_msgs::msg::CanMsgData>(m_canMsgDataTSC1);
  {
    std::lock_guard<std::mutex> guard(m_lockTSC1);
    if (Msg2RawData(msg, m_canDataTSC1) != ErrorCode::OK)
    {
      LOG_ERROR() << "Pack m_canDataTSC1 Error, name: " << msg->msg_name.c_str()
                  << "id: " << msg->msg_id
                  << "pgn: " << Byte::PGNFromCanId(msg->msg_id).c_str();
      return false;
    }
  }
  m_pubRawCanData->publish(m_canDataTSC1);
  return true;
}

bool CanMsgParserPackNode::PubVehicleCanMsg2Ego()
{
  m_iCnt10ms = (m_iCnt10ms + 10) % 1000;
  for (auto &iter : m_indexMsgTimeout)
  {
    if (!(m_iCnt10ms % iter.second[1]))
    {
      iter.second[2]++;
    }
    if (iter.second[2] > 5) // 5 timeout
    {
      m_vCntMsgTimeout[iter.second[0]] = NS_ZF::ECanMsgStatus::ERROR;
    }
    else
    {
      m_vCntMsgTimeout[iter.second[0]] = NS_ZF::ECanMsgStatus::OK;
    }
    // LOG_DEBUG() << "---" << iter.second[0] << "---" << iter.second[1] << "---" << iter.second[2];
  }
  // m_sMsgTimeout.assign(m_vCntMsgTimeout.begin(), m_vCntMsgTimeout.end());
  m_canMsgData2Ego.msg_name.assign(m_vCntMsgTimeout.begin(), m_vCntMsgTimeout.end());
  m_canMsgData2Ego.header.stamp = this->now();
  if (m_canMsgData2Ego.has_msg_id)
  {
    m_canMsgData2Ego.has_msg_id = !m_canMsgData2Ego.has_msg_id;
    {
      std::lock_guard<std::mutex> guard(m_lockSigs2Ego);
      for (auto &sig : m_canMsgData2Ego.sig_datas)
      {
        sig.sig_data = m_mapSigs2Ego[sig.sig_name];
      }
    }
    m_pubMsgData->publish(m_canMsgData2Ego);
  }
  else
  {
    m_canMsgData2Ego.has_msg_id = !m_canMsgData2Ego.has_msg_id;
  }
  return true;
}

bool CanMsgParserPackNode::PubAEBS2_27ForAC1000T()
{
  /*
    DrvrActvtnDmndFrAdvncdEmrgncyBrk 3 "dont' care / take no action"
    2 "reserved"
    1 "the driver wants the Advanced Emergency Braking System to warn or
    intervene if necessary" 0 "the driver does not want the Advanced Emergency
    Braking System to warn or intervene at any time" ;
  */
  m_canMsgDataAEBS2_27.header.stamp = this->now();
  const msg::CanMsgData::ConstSharedPtr msg =
      std::make_shared<const can_msgs::msg::CanMsgData>(m_canMsgDataAEBS2_27);
  if (Msg2RawData(msg, m_canDataAEBS2_27_AC1000T) != ErrorCode::OK)
  {
    LOG_ERROR() << "Pack AC1000T MsgData Error, name: " << msg->msg_name.c_str()
                << "id: " << msg->msg_id
                << "pgn: " << Byte::PGNFromCanId(msg->msg_id).c_str();
    return false;
  }
  m_pubRawCanData->publish(m_canDataAEBS2_27_AC1000T);
  return true;
}

bool CanMsgParserPackNode::ModifyDataTo5G4T(can_msgs::msg::Frame &msg,
                                            const std::string &pgn)
{
  if (!pgn.compare(gk_pgnTCO1))
  {
    // msg.data[3] = 0x40 & msg.data[3];
    msg.data[3] = 0x00 & msg.data[3];
    return true; // data changed
  }
  else if (!pgn.compare(gk_pgnVDC2))
  {
    msg.data[2] = 0x20; // turn counter to 0
    // uint32_t steerWheelAngle = (msg.data[1] << 8) + msg.data[0];
    // if (steerWheelAngle > 0x7D7F)
    // {
    //   msg.data[2] = 0xFF & 0x21 & msg.data[2];
    //   return true; // data changed
    // }
  }
  return false;
}

std::string CanMsgParserPackNode::LogString(
    const can_msgs::msg::CanMsgData &msgData)
{
  std::stringstream output_stream("");
  std_msgs::msg::Header head;
  head.stamp = this->now();
  output_stream << m_strInterface << ", " << "[" << head.stamp.sec << "."
                << std::setw(9) << std::setfill('0') << head.stamp.nanosec
                << "], " << "[" << msgData.header.stamp.sec << "."
                << std::setw(9) << std::setfill('0')
                << msgData.header.stamp.nanosec << "], " << msgData.msg_name
                << ", " << msgData.msg_pgn << ", "
                << Byte::Int2Hex(msgData.msg_id) << ", ";
  for (auto &sig : msgData.sig_datas)
  {
    output_stream << sig.sig_name << ", " << sig.sig_data << ", ";
  }
  return output_stream.str();
}

void CanMsgParserPackNode::InitSet()
{
  //----
  // std::set<std::string> pgnAC1000T = {
  //     // gk_pgnCCVS1,
  //     // gk_pgnEBC2_0B,
  //     // gk_pgnEBC5_0B,
  //     // gk_pgnERC1_29,
  //     // gk_pgnERC1_0F,
  //     // gk_pgnERC1_10,
  //     // gk_pgnTD_EE,
  //     // gk_pgnTCO1,
  //     gk_pgnVDC1,
  //     // gk_pgnETC1,
  //     // gk_pgnETC2_03,
  //     // gk_pgnVDC2_0B,
  //     // gk_pgnHRW_0B,
  //     gk_pgnVDHR_EE,
  //     // gk_pgnCCSS,
  //     gk_pgnOEL_32,
  //     gk_pgnOWW_32,
  //     // gk_pgnCVW_0B,
  //     // gk_pgnCVW_03,
  //     // gk_pgnEEC1,
  //     // gk_pgnEEC2,
  // };
  std::set<std::string> pgnAC1000T = {
      // gk_pgnCCVS1,
      // gk_pgnEBC2_0B,
      // gk_pgnEBC5_0B,
      // gk_pgnERC1_29, // NO  98FE5127
      // gk_pgnERC1_0F,
      // gk_pgnERC1_10,
      // gk_pgnTD_EE, // NO
      // gk_pgnTCO1,
      // gk_pgnVDC1,
      // gk_pgnETC1,
      // gk_pgnETC2_03,
      // gk_pgnVDC2_0B,
      // gk_pgnHRW_0B,
      // gk_pgnVDHR_EE, // NO
      // gk_pgnCCSS,    // NO
      // gk_pgnOEL_32, //
      // gk_pgnOWW_32,//NO
      // gk_pgnCVW_0B,
      // gk_pgnCVW_03,
      // gk_pgnEC1,// NO TP_DT
      // gk_pgnEEC1,
      // gk_pgnEEC2,
  };
  std::set<std::string> pgn5G4T = {
      gk_pgnTCO1, // TODO: comment
      gk_pgnEBC2, gk_pgnTD_EE, gk_pgnETC1, gk_pgnETC2_03, gk_pgnVDC2_0B,
      gk_pgnOEL_32, // 8CFD CC21;2365443105
      gk_pgnOWW_32, gk_pgnCCVS1, gk_pgnVDHR_EE, gk_pgnEEC1};
  m_mapPgnSet[CANCardParameter::CANDescription::CAN_DES_AC1000T] = pgnAC1000T;
  m_mapPgnSet[CANCardParameter::CANDescription::CAN_DES_5G4T] = pgn5G4T;
}

void CanMsgParserPackNode::InitVehicleVars()
{
  m_isVehicleCan = true;
  m_canMsgData2Ego.sig_datas.clear();
  for (const auto &signal : gk_egoSignals)
  {
    m_mapSigsVehicle[static_cast<uint32_t>(signal.id)].push_back(signal);
    m_mapSigs2Ego[signal.alias] = 0.0;
    can_msgs::msg::CanSigData tData;
    tData.sig_name = signal.alias;
    tData.sig_data = 0.0;
    m_canMsgData2Ego.sig_datas.push_back(tData);
  }
  m_mapTransferIds = {
      {0x18FDC40B, ETransNode::TRANS2_EGO},       // EBC5_EBS
      {0x18FEE300, ETransNode::TRANS2_EGO},       // EC1
      {0x18ECFF00, ETransNode::TRANS2_EGO},       // EC1 EC00
      {0x18EBFF00, ETransNode::TRANS2_EGO},       // EC1 EB00
      {ECanId::EEC2, ETransNode::TRANS2_EGO},     // EEC2
      {ECanId::EEC3, ETransNode::TRANS2_EGO},     // EEC3
      {ECanId::ETC1, ETransNode::TRANS2_EGO},     // ETC1
      {ECanId::ETC2, ETransNode::TRANS2_EGO},     // ETC2
      {ECanId::HRW, ETransNode::TRANS2_EGO},      // HRW
      {ECanId::CVW_AMT, ETransNode::TRANS2_EGO},  // CVW_AMT
      {ECanId::CVW_EBS, ETransNode::TRANS2_EGO},  // CVW_EBS
      {ECanId::VDC2, ETransNode::TRANS2_EGO},     // VDC2
      {ECanId::TCO1, ETransNode::TRANS2_EGO},     // TCO1
      {ECanId::XBR_AEBS, ETransNode::TRANS2_EGO}, // XBR_AEBS
      {0x18FE4021, ETransNode::TRANS2_EGO},       // no LD
      {ECanId::OEL, ETransNode::TRANS2_EGO},      // OEL
      {ECanId::EBC1_EBS, ETransNode::TRANS2_EGO}, // EBC1_EBS
      {0x18FECA03, ETransNode::TRANS2_EGO},       // DM1_TCM
      {0x18FECA00, ETransNode::TRANS2_EGO},       // DM1_ENG
      {0x18FECA0B, ETransNode::TRANS2_EGO},       // DM1_EBS
      {0x18FF3181, ETransNode::TRANS2_UI},        // StwError2Orin
  };
  m_indexMsgTimeout = {
      {ECanId::EEC2, {0, 50, 0}},      // EEC2
      {ECanId::EEC3, {1, 250, 0}},     // EEC3
      {ECanId::ETC1, {2, 10, 0}},      // ETC1
      {ECanId::ETC2, {3, 100, 0}},     // ETC2
      {ECanId::HRW, {4, 20, 0}},       // HRW
      {ECanId::CVW_AMT, {5, 1000, 0}}, // CVW_AMT
      {ECanId::CVW_EBS, {6, 1000, 0}}, // CVW_EBS
      {ECanId::VDC2, {7, 10, 0}},      // VDC2
      {ECanId::TCO1, {8, 50, 0}},      // TCO1
      {ECanId::XBR_AEBS, {9, 200, 0}}, // XBR_AEBS
      {ECanId::OEL, {10, 1000, 0}},    // OEL
      {ECanId::EBC1_EBS, {11, 20, 0}}, // EBC1_EBS
  };
  // m_indexMsgTimeout = {
  //     {gk_pgnEEC2, {0, 50, 0}},
  //     {gk_pgnEEC3, {1, 250, 0}},
  //     {gk_pgnETC1, {2, 10, 0}},
  //     {gk_pgnETC2_03, {3, 100, 0}},
  //     {gk_pgnHRW_0B, {4, 20, 0}},
  //     {gk_pgnCVW_03, {5, 1000, 0}},//CVW_AMT
  //     {gk_pgnCVW_0B, {6, 1000, 0}},//CVW_EBS
  //     {gk_pgnVDC2_0B, {7, 10, 0}},
  //     {gk_pgnTCO1, {8, 50, 0}},
  //     {gk_pgnXBR, {9, 200, 0}},//XBR_AEBS
  //     {gk_pgnOEL_32, {10, 1000, 0}},
  //     {gk_pgnEBC1_EBS, {11, 20, 0}},
  // };
  std::vector<uint8_t> tCnt(m_indexMsgTimeout.size(), NS_ZF::ECanMsgStatus::ERROR);
  m_vCntMsgTimeout.swap(tCnt);
  m_canMsgData2Ego.msg_name.assign(m_vCntMsgTimeout.begin(), m_vCntMsgTimeout.end());
}

END_NS_ZF_DRIVER_CANBUS

RCLCPP_COMPONENTS_REGISTER_NODE(
    NS_ZF::NS_DRIVER::NS_CANBUS::CanMsgParserPackNode)