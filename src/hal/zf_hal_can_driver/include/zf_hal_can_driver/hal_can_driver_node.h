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
 * @brief Defines the CanClient receiver interface ros node.
 * @return
 */

#ifndef ZF_HAL_CAN_RECEIVER_H
#define ZF_HAL_CAN_RECEIVER_H

// #include <algorithm>
// #include <array>
#include <memory>
// #include <mutex>
#include <thread>
// #include <unordered_map>
// #include <vector>

#include "zf_hal_can_driver/client/can_client.h"
#include "zf_hal_can_driver/hal_can_driver_global.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
// #include "ros2_socketcan_msgs/msg/fd_frame.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

// /**
//  * @namespace NS_ZF::driver::canbus
//  */
BEGIN_NS_ZF_DRIVER_CANBUS

/// \brief CanDriverNode class which can pass messages
/// from CAN hardware or virtual channels
class HAL_CAN_DRIVER_PUBLIC CanDriverNode final
    : public lc::LifecycleNode
{
public:
    /// \brief Default constructor
    explicit CanDriverNode(rclcpp::NodeOptions options);

    /// \brief Callback from transition to "configuring" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_configure(const lc::State &state) override;

    /// \brief Callback from transition to "activating" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_activate(const lc::State &state) override;

    /// \brief Callback from transition to "deactivating" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_deactivate(const lc::State &state) override;

    /// \brief Callback from transition to "unconfigured" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_cleanup(const lc::State &state) override;

    /// \brief Callback from transition to "shutdown" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_shutdown(const lc::State &state) override;

    /// \brief Callback for reading from hardware interface on timer tick.
    void receive();
    void send(const can_msgs::msg::Frame::SharedPtr msg);

    void RosMsg2Can(const can_msgs::msg::Frame::SharedPtr msg, CanFrame &f);
    void Can2RosMsg(const CanFrame &f, can_msgs::msg::Frame &m);

private:
    std::string interface_;
    std::shared_ptr<lc::LifecyclePublisher<can_msgs::msg::Frame>> m_pubCanFrames;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_subCanFrames;

    // std::shared_ptr<lc::LifecyclePublisher<std_msgs::msg::String>> frames_pub_;
    // std::shared_ptr<lc::LifecyclePublisher<ros2_socketcan_msgs::msg::FdFrame>> fd_frames_pub_;
    std::unique_ptr<CanClient> m_canClient;
    std::unique_ptr<std::thread> m_threadRecv;
    std::chrono::nanoseconds m_timeout_ns;
    bool enable_fd_;
    bool enable_debug_;
    bool use_bus_time_;
    bool m_bIs5G4T;
    int m_iLogCnt;
    uint32_t m_iFramenNumRecv; // zero means rad all buffer in can controller
    uint16_t m_iBaudrate;
    std::shared_ptr<lc::LifecyclePublisher<std_msgs::msg::UInt32>> m_pubNodeStatus;
    rclcpp::TimerBase::SharedPtr m_timerNodeSataus;
    std_msgs::msg::UInt32 m_statusMsg;
};

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_CAN_RECEIVER_H