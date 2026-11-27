// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/fetch_subscription.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace qrb::adapter;
using Point = geometry_msgs::msg::Point;

namespace
{

class FetchSubscriptionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static void Publish(rclcpp::Publisher<Point> & publisher, double x)
  {
    Point point;
    point.x = x;
    publisher.publish(point);
  }

  static double TakeX(FetchSubscription<Point> & subscription)
  {
    const std::shared_ptr<Point> point = subscription.Take();
    return point == nullptr ? -1.0 : point->x;
  }
};

}  // namespace

// The messages wait in the KEEP_LAST history until taken, one per Take, oldest first.
TEST_F(FetchSubscriptionTest, IntraProcessHistoryWaitsUntilTaken)
{
  auto node = std::make_shared<rclcpp::Node>(
      "fetch_subscription_intra", rclcpp::NodeOptions().use_intra_process_comms(true));
  const rclcpp::QoS qos{rclcpp::KeepLast(2)};
  FetchSubscription<Point> subscription;
  subscription.Open(*node, "fetch_subscription_intra", qos);
  auto publisher = node->create_publisher<Point>("fetch_subscription_intra", qos);
  EXPECT_EQ(TakeX(subscription), -1.0);

  Publish(*publisher, 1.0);
  Publish(*publisher, 2.0);
  Publish(*publisher, 3.0);
  EXPECT_EQ(TakeX(subscription), 2.0);
  EXPECT_EQ(TakeX(subscription), 3.0);
  EXPECT_EQ(TakeX(subscription), -1.0);

  Publish(*publisher, 4.0);
  subscription.Close();
  EXPECT_FALSE(subscription.IsOpen());
}

// The same through the middleware, from a node without intra-process communication.
TEST_F(FetchSubscriptionTest, MiddlewareHistoryWaitsUntilTaken)
{
  auto node = std::make_shared<rclcpp::Node>(
      "fetch_subscription_sub", rclcpp::NodeOptions().use_intra_process_comms(true));
  auto source = std::make_shared<rclcpp::Node>("fetch_subscription_pub");
  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
  FetchSubscription<Point> subscription;
  subscription.Open(*node, "fetch_subscription_dds", qos);
  auto publisher = source->create_publisher<Point>("fetch_subscription_dds", qos);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline)
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(publisher->get_subscription_count(), 1u);

  Publish(*publisher, 1.0);
  Publish(*publisher, 2.0);
  Publish(*publisher, 3.0);
  ASSERT_TRUE(publisher->wait_for_all_acked(std::chrono::seconds(5)));
  EXPECT_EQ(TakeX(subscription), 2.0);
  EXPECT_EQ(TakeX(subscription), 3.0);
  EXPECT_EQ(TakeX(subscription), -1.0);
}
