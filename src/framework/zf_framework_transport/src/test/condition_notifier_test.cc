/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "zf_framework_transport/shm/condition_notifier.h"

#include "gtest/gtest.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

TEST(ConditionNotifierTest, constructor) {
  zf::framework::ConditionNotifier &notifier = ConditionNotifier::Instance();
  EXPECT_NE(&notifier, nullptr);
  // notifier.Shutdown();
}

TEST(ConditionNotifierTest, notify_listen) {
  auto &notifier = ConditionNotifier::Instance();
  ReadableInfo readable_info;
  while (notifier.Listen(100, &readable_info)) {
  }
  EXPECT_FALSE(notifier.Listen(100, &readable_info));
  EXPECT_TRUE(notifier.Notify(readable_info));
  EXPECT_TRUE(notifier.Listen(100, &readable_info));
  EXPECT_FALSE(notifier.Listen(100, &readable_info));
  EXPECT_TRUE(notifier.Notify(readable_info));
  EXPECT_TRUE(notifier.Notify(readable_info));
  EXPECT_TRUE(notifier.Listen(100, &readable_info));
  EXPECT_TRUE(notifier.Listen(100, &readable_info));
  EXPECT_FALSE(notifier.Listen(100, &readable_info));
}

TEST(ConditionNotifierTest, notify_listen_readable_info) {
  auto &notifier = ConditionNotifier::Instance();
  ReadableInfo readable_info;
  readable_info.set_channel_id(1);
  readable_info.set_block_index(5);
  readable_info.set_host_id(7);

  // while (notifier.Listen(100, &readable_info)) {
  // }
  EXPECT_FALSE(notifier.Listen(100, &readable_info));

  EXPECT_TRUE(notifier.Notify(readable_info));
  ReadableInfo listen_info;
  EXPECT_TRUE(notifier.Listen(100, &listen_info));
  LOG_INFO() << "listen_info channel_id " << listen_info.channel_id();
  LOG_INFO() << "listen_info block_index " << listen_info.block_index();
  LOG_INFO() << "listen_info host_id " << listen_info.host_id();
  EXPECT_EQ(1, listen_info.channel_id());
  EXPECT_EQ(5, listen_info.block_index());
  EXPECT_EQ(7, listen_info.host_id());
}

TEST(ConditionNotifierTest, shutdown) {
  auto &notifier = ConditionNotifier::Instance();
  notifier.Shutdown();
  ReadableInfo readable_info;
  EXPECT_FALSE(notifier.Notify(readable_info));
  EXPECT_FALSE(notifier.Listen(100, &readable_info));
}

END_NS_ZF_FRAMEWORK
