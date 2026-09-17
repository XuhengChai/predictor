/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai@zf.com
 *
 *****************************************************************************/

/**
 * @file
 * @brief Abandoned.
 */

#ifndef ZF_HAL_CAN_RECEIVER_H
#define ZF_HAL_CAN_RECEIVER_H

#include <algorithm>
#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "zf_hal_can_driver/client/can_client.h"
#include "zf_hal_can_driver/hal_can_driver_global.h"
#include "zf_global/common/zf_global_async_util.h"
#include "zf_global/common/zf_global_macros.h"

/**
 * @namespace NS_ZF::driver::canbus
 */
BEGIN_NS_ZF_DRIVER_CANBUS

/**
 * @class CanReceiver
 * @brief CAN receiver.
 */
template <typename SensorType>
class CanReceiver
{
public:
  /**
   * @brief Constructor.
   */
  CanReceiver() = default;

  /**
   * @brief Destructor.
   */
  virtual ~CanReceiver() = default;

  /**
   * @brief Initialize by a CAN client, message manager.
   * @param can_client The CAN client to use for receiving messages.
   * @param pt_manager The message manager which can parse and
   *        get protocol data by message id.
   * @param enable_log If log the essential information during running.
   * @return An error code indicating the status of this initialization.
   */
  // NS_ZF::ErrorCode Init(CanClient *can_client,
  //                        MessageManager<SensorType> *pt_manager,
  //                        bool enable_log);
  NS_ZF::ErrorCode Init(std::unique_ptr<CanClient> client, bool enable_log);

  /**
   * @brief Get the working status of this CAN receiver.
   *        To check if it is running.
   * @return If this CAN receiver is running.
   */
  bool IsRunning() const;

  /**
   * @brief Start the CAN receiver.
   * @return The error code indicating the status of this action.
   */
  NS_ZF::ErrorCode Start();

  /**
   * @brief Stop the CAN receiver.
   */
  void Stop();

private:
  void RecvThreadFunc();

  int32_t Start(bool is_blocked);

private:
  std::atomic<bool> is_running_ = {false};
  // CanClient, MessageManager pointer life is managed by outer program
  std::unique_ptr<CanClient> can_client_ = nullptr;
  // CanClient *can_client_ = nullptr;
  // MessageManager<SensorType> *pt_manager_ = nullptr;
  bool enable_log_ = false;
  bool is_init_ = false;
  std::future<void> async_result_;
  DISALLOW_COPY_AND_ASSIGN(CanReceiver)
};

// template <typename SensorType>
// NS_ZF::ErrorCode CanReceiver<SensorType>::Init(
//     CanClient *can_client, MessageManager<SensorType> *pt_manager,
//     bool enable_log)
// {
//   can_client_ = can_client;
//   pt_manager_ = pt_manager;
//   enable_log_ = enable_log;
//   if (can_client_ == nullptr)
//   {
//     LOG_ERROR() << "Invalid can client.";
//     return NS_ZF::ErrorCode::CANBUS_ERROR;
//   }
//   if (pt_manager_ == nullptr)
//   {
//     LOG_ERROR() << "Invalid protocol manager.";
//     return NS_ZF::ErrorCode::CANBUS_ERROR;
//   }
//   is_init_ = true;
//   return NS_ZF::ErrorCode::OK;
// }
template <typename SensorType>
NS_ZF::ErrorCode CanReceiver<SensorType>::Init(
    std::unique_ptr<CanClient> client, bool enable_log)
{
  can_client_ = std::move(client);
  enable_log_ = enable_log;
  if (can_client_ == nullptr)
  {
    LOG_ERROR() << "Invalid can client.";
    return NS_ZF::ErrorCode::CANBUS_ERROR;
  }
  can_client_->Open();
  is_init_ = true;
  return NS_ZF::ErrorCode::OK;
}

template <typename SensorType>
void CanReceiver<SensorType>::RecvThreadFunc()
{
  LOG_INFO() << "Can client receiver thread starts.";
  if (can_client_ == nullptr)
  {
    LOG_ERROR() << "Invalid can client.";
  }

  int32_t receive_error_count = 0;
  int32_t receive_none_count = 0;
  const int32_t ERROR_COUNT_MAX = 10;
  auto default_period = 100 * 1000;//100ms

  while (IsRunning())
  {
    std::vector<CanFrame> buf;
    int32_t frame_num = MAX_CAN_RECV_FRAME_LEN;
    if (can_client_->Receive(&buf, frame_num) !=
        NS_ZF::ErrorCode::OK)
    {
      if (receive_error_count++ > ERROR_COUNT_MAX)
      {
        LOG_ERROR() << "Received " << receive_error_count << " error messages.";
      }
      NS_ZF::USleep(default_period);
      continue;
    }
    receive_error_count = 0;

    if (buf.size() != static_cast<size_t>(frame_num))
    {
      LOG_ERROR() << "Receiver buf size [" << buf.size()
                  << "] does not match can_client returned length["
                  << frame_num << "].";
    }

    if (frame_num == 0)
    {
      if (receive_none_count++ > ERROR_COUNT_MAX)
      {
        LOG_ERROR() << "Received " << receive_none_count << " empty messages.";
      }

      NS_ZF::USleep(default_period);
      continue;
    }
    receive_none_count = 0;

    for (const auto &frame : buf)
    {
      uint8_t len = frame.len;
      uint32_t uid = frame.id;
      const uint8_t *data = frame.data;
      // pt_manager_->Parse(uid, data, len);
      if (enable_log_)
      {
        LOG_DEBUG() << "recv_can_frame#" << frame.CanFrameString();
      }
    }
    NS_ZF::Yield();
  }
  LOG_INFO() << "Can client receiver thread stopped.";
}

template <typename SensorType>
bool CanReceiver<SensorType>::IsRunning() const
{
  return is_running_.load();
}

template <typename SensorType>
NS_ZF::ErrorCode CanReceiver<SensorType>::Start()
{
  if (is_init_ == false)
  {
    return NS_ZF::ErrorCode::CANBUS_ERROR;
  }
  is_running_.exchange(true);

  async_result_ = NS_ZF::Async(&CanReceiver<SensorType>::RecvThreadFunc, this);
  return NS_ZF::ErrorCode::OK;
}

template <typename SensorType>
void CanReceiver<SensorType>::Stop()
{
  if (IsRunning())
  {
    LOG_INFO() << "Stopping can client receiver ...";
    is_running_.exchange(false);
    async_result_.wait();
  }
  else
  {
    LOG_INFO() << "Can client receiver is not running.";
  }
  LOG_INFO() << "Can client receiver stopped [ok].";
}

END_NS_ZF_DRIVER_CANBUS

#endif // ZF_HAL_CAN_RECEIVER_H