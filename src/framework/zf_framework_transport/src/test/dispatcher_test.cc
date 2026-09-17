

#include <memory>
#include <vector>
#include "gtest/gtest.h"

#include "zf_framework_transport/message/identity.h"
#include "zf_framework_transport/dispatcher/dispatcher.h"
#include "zf_global/util/string_util.h"

BEGIN_NS_ZF_FRAMEWORK // zf::framework

    class DispatcherTest : public ::testing::Test
{
protected:
  DispatcherTest() : attr_num_(100)
  {
    for (int i = 0; i < attr_num_; ++i)
    {
      auto channel_name = "channel_" + std::to_string(i);
      RoleAttributes attr;
      attr.SetChannelName(channel_name);
      Identity self_id;
      attr.SetId(self_id.HashValue());
      self_attrs_.emplace_back(attr);

      Identity oppo_id;
      oppo_attrs_.emplace_back(attr);
      attr.SetId(oppo_id.HashValue());
    }
  }

  virtual ~DispatcherTest()
  {
    self_attrs_.clear();
    oppo_attrs_.clear();
    dispatcher_.Shutdown();
  }

  virtual void SetUp()
  {
    for (int i = 0; i < attr_num_; ++i)
    {
      dispatcher_.AddListener<MsgBase>(
          self_attrs_[i],
          [](const std::shared_ptr<MsgBase> &, const MessageInfo &) {});

      dispatcher_.AddListener<MsgBase>(
          self_attrs_[i],
          [](const std::shared_ptr<MsgBase> &, const MessageInfo &) {}, oppo_attrs_[i]);
    }
  }

  virtual void TearDown()
  {
    for (int i = 0; i < attr_num_; ++i)
    {
      dispatcher_.RemoveListener<MsgBase>(self_attrs_[i]);
      dispatcher_.RemoveListener<MsgBase>(self_attrs_[i],
                                          oppo_attrs_[i]);
    }
  }

  int attr_num_;
  Dispatcher dispatcher_;
  std::vector<RoleAttributes> self_attrs_;
  std::vector<RoleAttributes> oppo_attrs_;
};

TEST_F(DispatcherTest, shutdown) { dispatcher_.Shutdown(); }

TEST_F(DispatcherTest, add_and_remove_listener)
{
  RoleAttributes self_attr;
  self_attr.SetChannelName("add_listener");
  Identity self_id;
  self_attr.SetId(self_id.HashValue());

  RoleAttributes oppo_attr;
  oppo_attr.SetChannelName("add_listener");
  Identity oppo_id;
  oppo_attr.SetId(oppo_id.HashValue());

  dispatcher_.RemoveListener<MsgBase>(self_attr);
  dispatcher_.RemoveListener<MsgBase>(self_attr, oppo_attr);

  dispatcher_.AddListener<MsgBase>(
      self_attr, [](const std::shared_ptr<MsgBase> &,
                    const MessageInfo &)
      { LOG_INFO() << "I'm listener a."; });
  EXPECT_TRUE(dispatcher_.HasChannel(NS_ZF_STRING_UTIL::Hash("add_listener")));

  dispatcher_.AddListener<MsgBase>(
      self_attr, [](const std::shared_ptr<MsgBase> &,
                    const MessageInfo &)
      { LOG_INFO() << "I'm listener b."; });
  dispatcher_.RemoveListener<MsgBase>(self_attr);
  dispatcher_.Shutdown();

  dispatcher_.AddListener<MsgBase>(
      self_attr,
      [](const std::shared_ptr<MsgBase> &, const MessageInfo &)
      {
        LOG_INFO() << "I'm listener c.";
      },
      oppo_attr);

  dispatcher_.AddListener<MsgBase>(
      self_attr,
      [](const std::shared_ptr<MsgBase> &, const MessageInfo &)
      {
        LOG_INFO() << "I'm listener d.";
      },
      oppo_attr);

  dispatcher_.RemoveListener<MsgBase>(self_attr, oppo_attr);
}

TEST_F(DispatcherTest, has_channel)
{
  for (int i = 0; i < attr_num_; ++i)
  {
    auto channel_name = "channel_" + std::to_string(i);
    EXPECT_TRUE(dispatcher_.HasChannel(NS_ZF_STRING_UTIL::Hash(channel_name)));
  }
  EXPECT_FALSE(dispatcher_.HasChannel(NS_ZF_STRING_UTIL::Hash("has_channel")));
}

END_NS_ZF_FRAMEWORK
