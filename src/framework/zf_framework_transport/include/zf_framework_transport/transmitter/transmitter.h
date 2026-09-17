/******************************************************************************
 * Copyright 2023 ZF. All Rights Reserved.
 * Author:
 *        xuheng.chai
 * Licensed under the Apache License, Version 2.0 (the "License")
 *****************************************************************************/
/**
 * @file
 * @brief Defines the hal datas sync ros node.
 * @return
 */

#ifndef ZF_TRANSPORT_TRANSMITTER_TRANSMITTER_H_
#define ZF_TRANSPORT_TRANSMITTER_TRANSMITTER_H_

#include <cstdint>
#include <memory>
#include <string>

#include "zf_framework_transport/message/message_info.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

template <typename M>
class Transmitter
{
public:
  using MessagePtr = std::shared_ptr<M>;

  explicit Transmitter();
  virtual ~Transmitter();

  virtual void Enable() = 0;
  virtual void Disable() = 0;

  virtual bool Transmit(const MessagePtr &msg);
  virtual bool Transmit(const MessagePtr &msg, const MessageInfo &msg_info) = 0;
  const Identity &id() const { return id_; }
  uint64_t NextSeqNum() { return ++seq_num_; }
  uint64_t seq_num() const { return seq_num_; }

protected:
  uint64_t seq_num_;
  MessageInfo msg_info_;
  bool enabled_ = {};
  Identity id_;
};

template <typename M>
Transmitter<M>::Transmitter()
    : enabled_(false), id_(), seq_num_(0)
{
  msg_info_.set_sender_id(this->id_);
  msg_info_.set_seq_num(this->seq_num_);
}

template <typename M>
Transmitter<M>::~Transmitter() {}

template <typename M>
bool Transmitter<M>::Transmit(const MessagePtr &msg)
{
  msg_info_.set_seq_num(NextSeqNum());
  return Transmit(msg, msg_info_);
}

END_NS_ZF_FRAMEWORK

#endif // ZF_TRANSPORT_TRANSMITTER_TRANSMITTER_H_
