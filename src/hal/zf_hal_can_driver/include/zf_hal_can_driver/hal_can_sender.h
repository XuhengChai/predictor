// /******************************************************************************
//  * Copyright 2023 ZF. All Rights Reserved.
//  *****************************************************************************/

// /**
//  * @file
//  * @brief Defines the CanClient sender interface.
//  */

// #ifndef ZF_HAL_CAN_SENDER_H
// #define ZF_HAL_CAN_SENDER_H

// #include <algorithm>
// #include <array>
// #include <memory>
// #include <mutex>
// #include <thread>
// #include <unordered_map>
// #include <vector>

// #include "zf_hal_can_driver/client/can_client.h"
// #include "zf_hal_can_driver/hal_can_driver_global.h"


// /**
//  * @namespace NS_ZF::driver::canbus
//  */
// BEGIN_NS_ZF_DRIVER_CANBUS

// /**
//  * @class SenderMessage
//  * @brief This class defines the message to send.
//  */
// template <typename SensorType>
// class SenderMessage {
//  public:
//   /**
//    * @brief Constructor which takes message ID and protocol data.
//    * @param message_id The message ID.
//    * @param protocol_data A pointer of ProtocolData
//    *        which contains the content to send.
//    */
//   SenderMessage(const uint32_t message_id,
//                 ProtocolData<SensorType> *protocol_data);

//   /**
//    * @brief Constructor which takes message ID and protocol data and
//    *        and indicator whether to initialize all bits of the .
//    * @param message_id The message ID.
//    * @param protocol_data A pointer of ProtocolData
//    *        which contains the content to send.
//    * @param init_with_one If it is true, then initialize all bits in
//    *        the protocol data as one.
//    */
//   SenderMessage(const uint32_t message_id,
//                 ProtocolData<SensorType> *protocol_data, bool init_with_one);

//   /**
//    * @brief Destructor.
//    */
//   virtual ~SenderMessage() = default;

//   /**
//    * @brief Update the current period for sending messages by a difference.
//    * @param delta_period Update the current period by reducing delta_period.
//    */
//   void UpdateCurrPeriod(const int32_t delta_period);

//   /**
//    * @brief Update the protocol data. But the updating process depends on
//    *        the real type of protocol data which inherites ProtocolData.
//    */
//   void Update();

//   /**
//    * @brief Always update the protocol data. But the updating process depends on
//    *        the real type of protocol data which inherites ProtocolData.
//    */
//   void Update_Heartbeat();

//   /**
//    * @brief Get the CAN frame to send.
//    * @return The CAN frame to send.
//    */
//   struct CanFrame CanFrame();

//   /**
//    * @brief Get the message ID.
//    * @return The message ID.
//    */
//   uint32_t message_id() const;

//   /**
//    * @brief Get the current period to send messages. It may be different from
//    *        the period from protocol data by updating.
//    * @return The current period.
//    */
//   int32_t curr_period() const;

//  private:
//   uint32_t message_id_ = 0;
//   ProtocolData<SensorType> *protocol_data_ = nullptr;

//   int32_t period_ = 0;
//   int32_t curr_period_ = 0;

//  private:
//   static std::mutex mutex_;
//   struct CanFrame can_frame_to_send_;
//   struct CanFrame can_frame_to_update_;
// };

// /**
//  * @class CanSender
//  * @brief CAN sender.
//  */
// template <typename SensorType>
// class CanSender {
//  public:
//   /**
//    * @brief Constructor.
//    */
//   CanSender() = default;

//   /**
//    * @brief Destructor.
//    */
//   virtual ~CanSender() = default;

//   /**
//    * @brief Initialize by a CAN client based on its brand.
//    * @param can_client The CAN client to use for sending messages.
//    * @param enable_log whether enable record the send can frame log
//    * @return An error code indicating the status of this initialization.
//    */
//   NS_ZF::ErrorCode Init(CanClient *can_client,
//                          MessageManager<SensorType> *pt_manager,
//                          bool enable_log);

//   /**
//    * @brief Add a message with its ID, protocol data.
//    * @param message_id The message ID.
//    * @param protocol_data A pointer of ProtocolData
//    *        which contains the content to send.
//    * @param init_with_one If it is true, then initialize all bits in
//    *        the protocol data as one. By default, it is false.
//    */
//   void AddMessage(uint32_t message_id, ProtocolData<SensorType> *protocol_data,
//                   bool init_with_one = false);

//   /**
//    * @brief Start the CAN sender.
//    * @return The error code indicating the status of this action.
//    */
//   apollo::NS_ZF::ErrorCode Start();

//   /*
//    * @brief Update the protocol data based the types.
//    */
//   void Update();

//   /*
//    * @brief Update the heartbeat protocol data based the types.
//    */
//   void Update_Heartbeat();

//   /**
//    * @brief Stop the CAN sender.
//    */
//   void Stop();

//   /**
//    * @brief Get the working status of this CAN sender.
//    *        To check if it is running.
//    * @return If this CAN sender is running.
//    */
//   bool IsRunning() const;
//   bool enable_log() const;

//   FRIEND_TEST(CanSenderTest, OneRunCase);

//  private:
//   void PowerSendThreadFunc();

//   bool NeedSend(const SenderMessage<SensorType> &msg,
//                 const int32_t delta_period);
//   bool is_init_ = false;
//   bool is_running_ = false;

// //   CanClient *can_client_ = nullptr;  // Owned by global canbus.cc
//   std::unique_ptr<CanClient> can_client_ = nullptr;

//   MessageManager<SensorType> *pt_manager_ = nullptr;
//   std::vector<SenderMessage<SensorType>> send_messages_;
//   std::unique_ptr<std::thread> thread_;
//   bool enable_log_ = false;

//   DISALLOW_COPY_AND_ASSIGN(CanSender);
// };

// const uint32_t kSenderInterval = 6000;

// template <typename SensorType>
// std::mutex SenderMessage<SensorType>::mutex_;

// template <typename SensorType>
// SenderMessage<SensorType>::SenderMessage(
//     const uint32_t message_id, ProtocolData<SensorType> *protocol_data)
//     : SenderMessage(message_id, protocol_data, false) {}

// template <typename SensorType>
// SenderMessage<SensorType>::SenderMessage(
//     const uint32_t message_id, ProtocolData<SensorType> *protocol_data,
//     bool init_with_one)
//     : message_id_(message_id), protocol_data_(protocol_data) {
//   if (init_with_one) {
//     for (int32_t i = 0; i < protocol_data->GetLength(); ++i) {
//       can_frame_to_update_.data[i] = 0xFF;
//     }
//   }
//   int32_t len = protocol_data_->GetLength();

//   can_frame_to_update_.id = message_id_;
//   can_frame_to_update_.len = static_cast<uint8_t>(len);

//   period_ = protocol_data_->GetPeriod();
//   curr_period_ = period_;

//   Update();
// }

// template <typename SensorType>
// void SenderMessage<SensorType>::UpdateCurrPeriod(const int32_t period_delta) {
//   curr_period_ -= period_delta;
//   if (curr_period_ <= 0) {
//     curr_period_ = period_;
//   }
// }

// template <typename SensorType>
// void SenderMessage<SensorType>::Update() {
//   if (protocol_data_ == nullptr) {
//     LOG_ERROR() <<  "Attention: ProtocolData is nullptr!";
//     return;
//   }
//   protocol_data_->UpdateData(can_frame_to_update_.data);

//   std::lock_guard<std::mutex> lock(mutex_);
//   can_frame_to_send_ = can_frame_to_update_;
// }

// template <typename SensorType>
// void SenderMessage<SensorType>::Update_Heartbeat() {
//   if (protocol_data_ == nullptr) {
//     LOG_ERROR() <<  "Attention: ProtocolData is nullptr!";
//     return;
//   }
//   protocol_data_->UpdateData_Heartbeat(can_frame_to_update_.data);

//   std::lock_guard<std::mutex> lock(mutex_);
//   can_frame_to_send_ = can_frame_to_update_;
// }

// template <typename SensorType>
// uint32_t SenderMessage<SensorType>::message_id() const {
//   return message_id_;
// }

// template <typename SensorType>
// struct CanFrame SenderMessage<SensorType>::CanFrame() {
//   std::lock_guard<std::mutex> lock(mutex_);
//   return can_frame_to_send_;
// }

// template <typename SensorType>
// int32_t SenderMessage<SensorType>::curr_period() const {
//   return curr_period_;
// }

// template <typename SensorType>
// void CanSender<SensorType>::PowerSendThreadFunc() {
//   CHECK_NOTNULL(can_client_);
//   sched_param sch;
//   sch.sched_priority = 99;
//   pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

//   const int32_t INIT_PERIOD = 5000;  // 5ms
//   int32_t delta_period = INIT_PERIOD;
//   int32_t new_delta_period = INIT_PERIOD;

//   int64_t tm_start = 0;
//   int64_t tm_end = 0;
//   int64_t sleep_interval = 0;

//   LOG_INFO() << "Can client sender thread starts.";

//   while (is_running_) {
//     tm_start = cyber::Time::Now().ToNanosecond() / 1e3;
//     new_delta_period = INIT_PERIOD;

//     for (auto &message : send_messages_) {
//       bool need_send = NeedSend(message, delta_period);
//       message.UpdateCurrPeriod(delta_period);
//       new_delta_period = std::min(new_delta_period, message.curr_period());

//       if (!need_send) {
//         continue;
//       }
//       std::vector<CanFrame> can_frames;
//       CanFrame can_frame = message.CanFrame();
//       can_frames.push_back(can_frame);
//       if (can_client_->SendSingleFrame(can_frames) != NS_ZF::ErrorCode::OK) {
//         LOG_ERROR() <<  "Send msg failed:" << can_frame.CanFrameString();
//       }
//       if (enable_log()) {
//         LOG_DEBUG() << "send_can_frame#" << can_frame.CanFrameString()
//                << "echo send_can_frame# in chssis_detail.";
//         uint32_t uid = can_frame.id;
//         const uint8_t *data = can_frame.data;
//         uint8_t len = can_frame.len;
//         pt_manager_->Parse(uid, data, len);
//       }
//     }
//     delta_period = new_delta_period;
//     tm_end = cyber::Time::Now().ToNanosecond() / 1e3;
//     sleep_interval = delta_period - (tm_end - tm_start);

//     if (sleep_interval > 0) {
//       std::this_thread::sleep_for(std::chrono::microseconds(sleep_interval));
//     } else {
//       // do not sleep
//       LOG_WARN() << "Too much time for calculation: " << tm_end - tm_start
//             << "us is more than minimum period: " << delta_period << "us";
//     }
//   }
//   LOG_INFO() << "Can client sender thread stopped!";
// }

// template <typename SensorType>
// NS_ZF::ErrorCode CanSender<SensorType>::Init(
//     CanClient *can_client, MessageManager<SensorType> *pt_manager,
//     bool enable_log) {
//   if (is_init_) {
//     LOG_ERROR() <<  "Duplicated Init request.";
//     return NS_ZF::ErrorCode::CANBUS_ERROR;
//   }
//   if (can_client == nullptr) {
//     LOG_ERROR() <<  "Invalid can client.";
//     return NS_ZF::ErrorCode::CANBUS_ERROR;
//   }
//   is_init_ = true;
//   can_client_ = can_client;
//   pt_manager_ = pt_manager;
//   enable_log_ = enable_log;
//   return NS_ZF::ErrorCode::OK;
// }

// template <typename SensorType>
// void CanSender<SensorType>::AddMessage(uint32_t message_id,
//                                        ProtocolData<SensorType> *protocol_data,
//                                        bool init_with_one) {
//   if (protocol_data == nullptr) {
//     LOG_ERROR() <<  "invalid protocol data.";
//     return;
//   }
//   send_messages_.emplace_back(
//       SenderMessage<SensorType>(message_id, protocol_data, init_with_one));
//   LOG_INFO() << "Add send message:" << std::hex << message_id;
// }

// template <typename SensorType>
// NS_ZF::ErrorCode CanSender<SensorType>::Start() {
//   if (is_running_) {
//     LOG_ERROR() <<  "Cansender has already started.";
//     return NS_ZF::ErrorCode::CANBUS_ERROR;
//   }
//   is_running_ = true;
//   thread_.reset(new std::thread([this] { PowerSendThreadFunc(); }));

//   return NS_ZF::ErrorCode::OK;
// }

// // cansender -> Update_Heartbeat()
// template <typename SensorType>
// void CanSender<SensorType>::Update_Heartbeat() {
//   for (auto &message : send_messages_) {
//     message.Update_Heartbeat();
//   }
// }

// template <typename SensorType>
// void CanSender<SensorType>::Update() {
//   for (auto &message : send_messages_) {
//     message.Update();
//   }
// }

// template <typename SensorType>
// void CanSender<SensorType>::Stop() {
//   if (is_running_) {
//     LOG_INFO() << "Stopping can sender ...";
//     is_running_ = false;
//     if (thread_ != nullptr && thread_->joinable()) {
//       thread_->join();
//     }
//     thread_.reset();
//   } else {
//     LOG_ERROR() <<  "CanSender is not running.";
//   }

//   LOG_INFO() << "Can client sender stopped [ok].";
// }

// template <typename SensorType>
// bool CanSender<SensorType>::IsRunning() const {
//   return is_running_;
// }

// template <typename SensorType>
// bool CanSender<SensorType>::enable_log() const {
//   return enable_log_;
// }

// template <typename SensorType>
// bool CanSender<SensorType>::NeedSend(const SenderMessage<SensorType> &msg,
//                                      const int32_t delta_period) {
//   return msg.curr_period() <= delta_period;
// }

// END_NS_ZF_DRIVER_CANBUS

// #endif // ZF_HAL_CAN_SENDER_H