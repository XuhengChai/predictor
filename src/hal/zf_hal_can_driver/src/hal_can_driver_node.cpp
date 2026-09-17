#include "zf_hal_can_driver/hal_can_driver_node.h"

#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <regex>

#include "rclcpp/time.hpp"
#include "zf_global/util/logger.h"
#include "zf_global/zf_global_topic_name.h"
#include "zf_hal_can_driver/client/can_client_factory.h"
#include "zf_hal_can_driver/hal_can_id.h"

// #include <chrono>
// #include <memory>
// #include <string>
// #include <utility>
// #include <vector>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;
using namespace std::chrono_literals;

BEGIN_NS_ZF_DRIVER_CANBUS

// LOGGER::FileLogger LOG_FILE_RAW("build_at_" __DATE__ "_" __TIME__ "Raw.log");
LOGGER::FileLogger LOG_FILE_RAW;

// Convert ROS2 time to socketcan timeval
timeval RosToTimeval(const rclcpp::Time &t)
{
    timeval tv;
    tv.tv_sec = static_cast<time_t>(t.seconds());
    tv.tv_usec = static_cast<suseconds_t>(t.nanoseconds());
    return tv;
}

// Convert socketcan timeval to ROS2 time
rclcpp::Time timevalToRos(const timeval &tv)
{
    // return rclcpp::Time(ns.count());
    return rclcpp::Time(tv.tv_sec, tv.tv_usec);
}

timeval StdtoTimeval(const std::chrono::nanoseconds timeout) noexcept
{
    const auto count = timeout.count();
    constexpr auto BILLION = 1'000'000'000LL;
    struct timeval c_timeout;
    c_timeout.tv_sec = static_cast<decltype(c_timeout.tv_sec)>(count / BILLION);
    c_timeout.tv_usec =
        static_cast<decltype(c_timeout.tv_usec)>((count % BILLION) / 1000LL);
    return c_timeout;
}

CanDriverNode::CanDriverNode(rclcpp::NodeOptions options)
    : lc::LifecycleNode("can_driver_node", options), m_iFramenNumRecv(1)
{
    interface_ = this->declare_parameter("interface", "can0");
    m_bIs5G4T = (!interface_.compare("can1"));
    use_bus_time_ = this->declare_parameter<bool>("use_bus_time", false);
    enable_fd_ = this->declare_parameter<bool>("enable_can_fd", false);
    enable_debug_ = this->declare_parameter<bool>("enable_debug", false);
    m_iLogCnt = this->declare_parameter("log_cnt", 0);
    m_iBaudrate = this->declare_parameter("baudrate", 0);
    double timeout_s = this->declare_parameter("timeout_sec", 0.2);
    this->declare_parameter("filters", "0:0");
    m_timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_s));

    RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());
    RCLCPP_INFO(this->get_logger(), "use bus time: %d", use_bus_time_);
    RCLCPP_INFO(this->get_logger(), "can fd enabled: %s",
                enable_fd_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "timeout_sec(s): %f", timeout_s);
    RCLCPP_INFO(this->get_logger(), "enable_debug_: %d", enable_debug_);
    RCLCPP_INFO(this->get_logger(), "m_bIs5G4T: %d", m_bIs5G4T);
    RCLCPP_INFO(this->get_logger(), "m_iLogCnt: %d", m_iLogCnt);
    // auto fileName = "build_" + std::to_string(m_iLogCnt) + "_at_" __DATE__ "_"
    // __TIME__ "raw.log";
    auto fileName =
        "build_" + std::to_string(m_iLogCnt) + "_at_" __DATE__ "_raw.log";
    LOG_FILE_RAW.SetFileName(fileName);
}

LNI::CallbackReturn CanDriverNode::on_configure(const lc::State &state)
{
    (void)state;
    rclcpp::QoS status_qos(500);
    status_qos.keep_last(500);
    // status_qos.reliable();
    status_qos.best_effort();
    status_qos.durability_volatile();
    try
    {
        LOG_INFO() << "on_configure: ";
        CANCardParameter param;
        if (this->interface_.find("pcan") != std::string::npos)
        {
            param.brand = CANCardParameter::CANCardBrand::PCAN;
            m_iFramenNumRecv = 0;
            std::regex pattern(R"(\d+)");
            std::smatch match;
            int id = 0;
            if (std::regex_search(this->interface_, match, pattern))
            {
                id = std::stoi(match[0]);
            }
            if (id < (CANCardParameter::CANChannelId::CHANNEL_ID_MAX + 1))
            {
                param.channelID = static_cast<CANCardParameter::CANChannelId>(id);
            }
            // param.brand = CANCardParameter::CANCardBrand::ETHERNET_CAN;
        }
        else if (this->interface_.find("can") != std::string::npos)
        {
            param.brand = CANCardParameter::CANCardBrand::SOCKET_CAN;
        }
        else if (this->interface_.find("eth") != std::string::npos)
        {
            param.brand = CANCardParameter::CANCardBrand::ETHERNET_CAN;
        }
        else
        {
            LOG_ERROR() << "Invalid can interface: " << this->interface_.c_str();
            m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::ERROR);
            return LNI::CallbackReturn::FAILURE;
        }
        LOG_INFO() << "can brand: " << param.brand;

        // param.brand = CANCardParameter::CANCardBrand::SOCKET_CAN;
        param.deviceName = this->interface_;
        param.enableFD = enable_fd_;
        param.baudrate = static_cast<CANCardParameter::CANBaudrate>(m_iBaudrate);
        param.timestamp = StdtoTimeval(m_timeout_ns);
        // RCLCPP_INFO(this->get_logger(), "param.timestamp: %d.%d",
        // param.timestamp.tv_sec, param.timestamp.tv_usec);
        CanClientFactory::Instance().RegisterCanClients();
        // m_canClient =
        // std::move(CanClientFactory::Instance().CreateCANClient(param));
        m_canClient = CanClientFactory::Instance().CreateCANClient(param);
        if (m_canClient == nullptr)
        {
            LOG_ERROR() << "Invalid can client.";
            m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::ERROR);
            return LNI::CallbackReturn::FAILURE;
        }

        auto ret = m_canClient->Open();
        if (ret != ErrorCode::OK)
        {
            LOG_ERROR() << "Open can client failed";
            m_statusMsg.data = static_cast<uint32_t>(ret);
            return LNI::CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(get_logger(), "Open ret %d", ret);

        // apply CAN filters
        auto filters = get_parameter("filters").as_string();
        m_canClient->SetFilters(filters);
        RCLCPP_INFO(get_logger(), "applied filters: %s", filters.c_str());

        std::string topicName =
            "/" + interface_ + gk_halNodeStatusCanRaw; // /node_status/can_raw
        m_pubNodeStatus =
            this->create_publisher<std_msgs::msg::UInt32>(topicName, 50);
        m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::INIT);
        m_timerNodeSataus = create_wall_timer(
            100ms, [this]()
            { m_pubNodeStatus->publish(m_statusMsg); });
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Error opening CAN receiver: %s - %s",
                     interface_.c_str(), ex.what());
        m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::ERROR);
        return LNI::CallbackReturn::FAILURE;
    }
    std::string topicName =
        "/" + interface_ + gk_halNodeStatusCanRaw; // /node_status/can_raw
    m_pubNodeStatus =
        this->create_publisher<std_msgs::msg::UInt32>(topicName, 50);
    m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::INIT);
    m_timerNodeSataus = create_wall_timer(
        100ms, [this]()
        { m_pubNodeStatus->publish(m_statusMsg); });
    RCLCPP_DEBUG(this->get_logger(), "Receiver successfully configured.");

    topicName = "/" + interface_ + gk_halFromCanRawBus; // from_can_raw_bus
    m_pubCanFrames =
        this->create_publisher<can_msgs::msg::Frame>(topicName, status_qos);
    topicName = "/" + interface_ + gk_halToCanRawBus;
    m_subCanFrames = this->create_subscription<can_msgs::msg::Frame>(
        topicName, status_qos,
        std::bind(&CanDriverNode::send, this, std::placeholders::_1));

    m_threadRecv = std::make_unique<std::thread>(&CanDriverNode::receive, this);
    m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::READY);
    return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanDriverNode::on_activate(const lc::State &state)
{
    (void)state;
    m_pubCanFrames->on_activate();
    m_pubNodeStatus->on_activate();
    RCLCPP_DEBUG(this->get_logger(), "Receiver activated.");
    return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanDriverNode::on_deactivate(const lc::State &state)
{
    (void)state;
    m_pubCanFrames->on_deactivate();
    m_pubNodeStatus->on_deactivate();
    RCLCPP_DEBUG(this->get_logger(), "Receiver deactivated.");
    return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanDriverNode::on_cleanup(const lc::State &state)
{
    (void)state;
    m_pubCanFrames.reset();
    if (m_threadRecv->joinable())
    {
        m_threadRecv->join();
    }
    RCLCPP_DEBUG(this->get_logger(), "Receiver cleaned up.");
    return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn CanDriverNode::on_shutdown(const lc::State &state)
{
    (void)state;
    RCLCPP_DEBUG(this->get_logger(), "Receiver shutting down.");
    return LNI::CallbackReturn::SUCCESS;
}

void CanDriverNode::receive()
{
    can_msgs::msg::Frame frame_msg(
        rosidl_runtime_cpp::MessageInitialization::ZERO);
    frame_msg.header.frame_id = "";
    int32_t receive_error_count = 0;
    int32_t receive_none_count = 0;
    const int32_t ERROR_COUNT_MAX = 10;
    if (this->get_current_state().id() != State::PRIMARY_STATE_ACTIVE)
    {
        LOG_WARN() << " state not PRIMARY_STATE_ACTIVE."
                   << this->get_current_state().id();
        std::this_thread::sleep_for(100ms);
    }
    while (rclcpp::ok())
    {
        // LOG_INFO() << rclcpp::ok() << " rclcpp::ok()" << int(this->get_current_state().id());
        try
        {
            // receive_id = m_canClient->receive(frame_msg.data.data(), interval_ns_);
            std::vector<CanFrame> buf;
            // int32_t frame_num = MAX_CAN_RECV_FRAME_LEN;
            // int32_t frame_num = 1;
            auto rsl = m_canClient->Receive(&buf, m_iFramenNumRecv);
            if (rsl == NS_ZF::ErrorCode::CAN_PARSE_IGNORE)
            {
                // LOG_DEBUG() << "Received CAN_PARSE_IGNORE";
                std::this_thread::sleep_for(500us);
                continue;
            }
            if (rsl != NS_ZF::ErrorCode::OK)
            {
                if (receive_error_count++ > ERROR_COUNT_MAX)
                {
                    LOG_WARN() << "Received " << receive_error_count << " error messages.";
                    m_statusMsg.data = static_cast<uint32_t>(rsl);
                }
                std::this_thread::sleep_for(100ms);
                continue;
            }
            if (buf.empty())
            {
                if (receive_none_count++ > ERROR_COUNT_MAX)
                {
                    LOG_WARN() << "Received " << receive_none_count << " empty messages.";
                    m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::DOWNGRADE);
                }
                std::this_thread::sleep_for(100ms);
                continue;
            }
            receive_none_count = 0;

            if (m_iFramenNumRecv && (buf.size() != static_cast<size_t>(m_iFramenNumRecv)))
            {
                LOG_WARN() << "Receiver buf size [" << buf.size()
                           << "] does not match can_client returned length["
                           << m_iFramenNumRecv << "].";
            }
            receive_error_count = 0;
            // LOG_FILE_RAW() << "Receiver buf size [" << buf.size() << "]";

            for (const auto &frame : buf)
            {
                // uint8_t len = frame.len;
                // uint32_t uid = frame.id;
                // const uint8_t *data = frame.data;
                // pt_manager_->Parse(uid, data, len);
                if (m_bIs5G4T && (frame.id & 0xF))
                {
                    if (enable_debug_)
                    {
                        LOG_FILE_RAW() << "[" << frame_msg.header.stamp.sec << "."
                                       << std::setw(9) << std::setfill('0')
                                       << frame_msg.header.stamp.nanosec << "]-"
                                       << frame.CanFrameString();
                    }
                    continue;
                }
                Can2RosMsg(frame, frame_msg);
                if (!use_bus_time_)
                {
                    frame_msg.header.stamp = this->now();
                }
                m_pubCanFrames->publish(std::move(frame_msg));
                if (enable_debug_)
                {
                    // LOG_FILE_RAW() << interface_ << ", " << frame.CanFrameString();
                    LOG_FILE_RAW() << "[" << frame_msg.header.stamp.sec << "."
                                   << std::setw(9) << std::setfill('0')
                                   << frame_msg.header.stamp.nanosec << "]-"
                                   << frame.CanFrameString();
                    // LOG_DEBUG() << "[" << frame_msg.header.stamp.sec << "." <<
                    // frame_msg.header.stamp.nanosec
                    //             << "]-" << frame.CanFrameString() << " recv and pub can_frame --";
                    // LOG_DEBUG() << frame.CanFrameString();
                }
            }
            // std::this_thread::sleep_for(1ms);
            // usleep(1000);
            m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::READY);
            std::this_thread::yield();
        }
        catch (const std::exception &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Error receiving CAN message: %s - %s",
                                 interface_.c_str(), ex.what());
            continue;
        }
    }
    m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::ERROR);
    RCLCPP_WARN(this->get_logger(), "Error receiving not ok ");
}

void CanDriverNode::send(const can_msgs::msg::Frame::SharedPtr msg)
{
    // if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE)
    // {
    // }
    try
    {
        CanFrame sendBuf;
        RosMsg2Can(msg, sendBuf);
        auto errCode = m_canClient->SendSingleFrame({sendBuf});
        if (errCode != NS_ZF::ErrorCode::OK)
        {
            m_statusMsg.data = static_cast<uint32_t>(ENodeStatus::ERROR);
            // m_statusMsg.data = errCode;
        };
        // if (enable_debug_)
        // {
        //     LOG_DEBUG()<< interface_.c_str() << ": send_can_frame#" <<
        //     sendBuf.CanFrameString() << "\n";
        // }
    }
    catch (const std::exception &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Error sending CAN message: %s - %s",
                             interface_.c_str(), ex.what());
        return;
    }
}

void CanDriverNode::RosMsg2Can(const can_msgs::msg::Frame::SharedPtr msg,
                               CanFrame &f)
{
    FrameType type;
    if (msg->is_rtr)
    {
        type = FrameType::REMOTE;
    }
    else if (msg->is_error)
    {
        type = FrameType::ERROR;
    }
    else
    {
        type = FrameType::DATA;
    }
    CanId send_id(msg->id, type, msg->is_extended);
    // LOG_DEBUG() << int(type)<< "---RosMsg2Can#" << msg->id << ", " <<
    // msg->is_extended << ", "<<  send_id.Id();
    f.id = send_id.IdRaw();
    f.len = msg->dlc;
    // f.is_error = m.is_error;
    // f.is_rtr = m.is_rtr;
    // f.is_extended = m.is_extended;
    f.timestamp.tv_sec = static_cast<time_t>(msg->header.stamp.sec);
    f.timestamp.tv_usec =
        static_cast<suseconds_t>(msg->header.stamp.nanosec / 1000);
    // for (int i = 0; i < CANBUS_MESSAGE_LENGTH; i++) // always copy all data,
    // regardless of dlc.
    // {
    //     f.data[i] = msg->data[i];
    // }
    std::memcpy(f.data, msg->data.data(), msg->dlc);
}

void CanDriverNode::Can2RosMsg(const CanFrame &f, can_msgs::msg::Frame &m)
{
    // m.id = f.id;
    CanId receive_id(f.id);
    m.id = receive_id.Id();
    m.dlc = f.len;
    m.is_error = (receive_id.GetFrameType() == FrameType::ERROR);
    m.is_rtr = (receive_id.GetFrameType() == FrameType::REMOTE);
    m.is_extended = receive_id.IsExtended();
    m.header.stamp.sec = static_cast<int32_t>(f.timestamp.tv_sec);
    m.header.stamp.nanosec = static_cast<uint32_t>(f.timestamp.tv_usec * 1000);
    m.data.clear();
    // for (int i = 0; i < CANBUS_MESSAGE_LENGTH; i++) // always copy all data,
    // regardless of dlc.
    // {
    //     m.data.push_back(f.data[i]);
    //     // m.data[i] = f.data[i];
    // }
    m.data.resize(f.len); // Resize the vector 'data' to accommodate the data
    memcpy(m.data.data(), f.data, f.len);
}

END_NS_ZF_DRIVER_CANBUS

RCLCPP_COMPONENTS_REGISTER_NODE(NS_ZF::NS_DRIVER::NS_CANBUS::CanDriverNode)
