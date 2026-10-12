// Copyright 2026 semantic-nav-ros contributors. SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "semantic_nav_costmap_layer/semantic_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl/time.h"

using semantic_nav_costmap_layer::SemanticLayer;

class TestLayer : public SemanticLayer
{
public:
  using SemanticLayer::receiveGrid;
};

class SemanticLayerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    options.arguments({"--ros-args", "-r", "/clock:=/semantic_layer_tests/clock"});
    node = std::make_shared<nav2_util::LifecycleNode>("semantic_layer_test", "", options);
    setTime(100);
    tf = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    map = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    map->resizeMap(4, 4, 1.0, 0.0, 0.0);
    group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer = std::make_shared<TestLayer>();
    layer->initialize(map.get(), "semantic_layer", tf.get(), node, group);
    map->addPlugin(layer);
  }
  void TearDown() override
  {
    map.reset();
    layer.reset();
    tf.reset();
    group.reset();
    node.reset();
  }
  void setTime(int seconds)
  {
    ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()));
    ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(
      node->get_clock()->get_clock_handle(), static_cast<int64_t>(seconds) * 1000000000));
  }
  nav2_msgs::msg::Costmap::SharedPtr grid()
  {
    auto msg = std::make_shared<nav2_msgs::msg::Costmap>();
    msg->header.frame_id = "map";
    msg->header.stamp = node->now();
    msg->metadata.size_x = 4;
    msg->metadata.size_y = 4;
    msg->metadata.resolution = 1.0;
    msg->metadata.origin.orientation.w = 1.0;
    msg->data.assign(16, 0);
    return msg;
  }
  nav2_util::LifecycleNode::SharedPtr node;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map;
  rclcpp::CallbackGroup::SharedPtr group;
  std::shared_ptr<TestLayer> layer;
};

TEST_F(SemanticLayerTest, GraduatedCostsPreserveExistingObstaclesAndUnknown)
{
  auto msg = grid();
  msg->data = {0, 40, 150, 254, 255, 20, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0};
  layer->receiveGrid(msg);
  auto master = map->getCostmap();
  master->setCost(1, 1, 254);
  master->setCost(2, 1, 255);
  master->setCost(3, 1, 200);
  layer->updateCosts(*master, 0, 0, 4, 4);
  EXPECT_EQ(master->getCost(0, 0), 0);
  EXPECT_EQ(master->getCost(1, 0), 40);
  EXPECT_EQ(master->getCost(2, 0), 150);
  EXPECT_EQ(master->getCost(3, 0), 254);
  EXPECT_EQ(master->getCost(0, 1), 0);
  EXPECT_EQ(master->getCost(1, 1), 254);
  EXPECT_EQ(master->getCost(2, 1), 255);
  EXPECT_EQ(master->getCost(3, 1), 200);
}

TEST_F(SemanticLayerTest, ReplacementsExpiryAndResetClearOldCosts)
{
  auto msg = grid();
  msg->data[0] = 254;
  layer->receiveGrid(msg);
  map->updateMap(0, 0, 0);
  ASSERT_EQ(map->getCostmap()->getCost(0, 0), 254);
  auto replacement = grid();
  replacement->data[15] = 80;
  layer->receiveGrid(replacement);
  map->updateMap(0, 0, 0);
  EXPECT_EQ(map->getCostmap()->getCost(0, 0), 0);
  EXPECT_EQ(map->getCostmap()->getCost(3, 3), 80);
  setTime(106);
  map->updateMap(0, 0, 0);
  EXPECT_EQ(map->getCostmap()->getCost(3, 3), 0);
  layer->receiveGrid(grid());
  layer->reset();
  map->updateMap(0, 0, 0);
  EXPECT_EQ(map->getCostmap()->getCost(3, 3), 0);
}

TEST_F(SemanticLayerTest, RejectsMalformedAndFutureData)
{
  auto bad = grid();
  bad->data = {254};
  layer->receiveGrid(bad);
  map->updateMap(0, 0, 0);
  EXPECT_EQ(map->getCostmap()->getCost(0, 0), 0);
  bad = grid();
  bad->metadata.resolution = 0.0;
  bad->data[0] = 254;
  layer->receiveGrid(bad);
  map->updateMap(0, 0, 0);
  EXPECT_EQ(map->getCostmap()->getCost(0, 0), 0);
  auto future = grid();
  future->header.stamp.sec = 101;
  future->data[0] = 254;
  layer->receiveGrid(future);
  map->updateMap(0, 0, 0);
  EXPECT_EQ(map->getCostmap()->getCost(0, 0), 0);
}

TEST_F(SemanticLayerTest, LatestTransformSupportsDifferentCostmapFrame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "semantic_frame";
  transform.child_frame_id = "map";
  transform.transform.translation.x = -1.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  auto msg = grid();
  msg->header.frame_id = "semantic_frame";
  msg->data[0] = 150;
  layer->receiveGrid(msg);
  map->updateMap(0, 0, 0);
  EXPECT_EQ(map->getCostmap()->getCost(0, 0), 0);
  EXPECT_EQ(map->getCostmap()->getCost(1, 0), 150);
}

TEST_F(SemanticLayerTest, RotatedOriginAndUpdateWindow)
{
  auto msg = grid();
  msg->metadata.origin.position.x = 4.0;
  msg->metadata.origin.position.y = 4.0;
  msg->metadata.origin.orientation.z = 1.0;
  msg->metadata.origin.orientation.w = 0.0;
  msg->data[0] = 150;
  layer->receiveGrid(msg);
  layer->updateCosts(*map->getCostmap(), 0, 0, 2, 2);
  EXPECT_EQ(map->getCostmap()->getCost(3, 3), 0);
  layer->updateCosts(*map->getCostmap(), 2, 2, 4, 4);
  EXPECT_EQ(map->getCostmap()->getCost(3, 3), 150);
}

TEST_F(SemanticLayerTest, LethalSemanticCellFeedsStockInflation)
{
  map->resizeMap(20, 20, 0.1, 0, 0);
  auto inflation = std::make_shared<nav2_costmap_2d::InflationLayer>();
  inflation->initialize(map.get(), "inflation_layer", tf.get(), node, group);
  map->addPlugin(inflation);
  auto msg = grid();
  msg->metadata.size_x = 20;
  msg->metadata.size_y = 20;
  msg->metadata.resolution = 0.1;
  msg->data.assign(400, 0);
  msg->data[10 * 20 + 10] = 254;
  layer->receiveGrid(msg);
  map->updateMap(0, 0, 0);
  EXPECT_EQ(map->getCostmap()->getCost(10, 10), 254);
  EXPECT_GT(map->getCostmap()->getCost(11, 10), 0);
  EXPECT_LT(map->getCostmap()->getCost(11, 10), 254);
  setTime(106);
  map->updateMap(0, 0, 0);
  EXPECT_EQ(map->getCostmap()->getCost(10, 10), 0);
  EXPECT_EQ(map->getCostmap()->getCost(11, 10), 0);
}

TEST_F(SemanticLayerTest, PluginLoadsAndReceivesPublishedGrid)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto loaded = loader.createSharedInstance("semantic_nav_costmap_layer::SemanticLayer");
  loaded->initialize(map.get(), "loaded_layer", tf.get(), node, group);
  auto publisher = node->create_publisher<nav2_msgs::msg::Costmap>(
    "/semantic_costmap", rclcpp::QoS(1).reliable().transient_local());
  publisher->on_activate();
  auto msg = grid();
  msg->data[5] = 123;
  publisher->publish(*msg);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline && map->getCostmap()->getCost(1, 1) != 123) {
    executor.spin_some();
    loaded->updateCosts(*map->getCostmap(), 0, 0, 4, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(map->getCostmap()->getCost(1, 1), 123);
  executor.remove_node(node->get_node_base_interface());
}
