// Copyright 2026 semantic-nav-ros contributors. SPDX-License-Identifier: Apache-2.0
#ifndef SEMANTIC_NAV_COSTMAP_LAYER__SEMANTIC_LAYER_HPP_
#define SEMANTIC_NAV_COSTMAP_LAYER__SEMANTIC_LAYER_HPP_

#include <mutex>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_msgs/msg/costmap.hpp"

namespace semantic_nav_costmap_layer
{
class SemanticLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  void reset() override;
  bool isClearable() override {return false;}

protected:
  void receiveGrid(nav2_msgs::msg::Costmap::ConstSharedPtr grid);

private:
  std::mutex mutex_;
  nav2_msgs::msg::Costmap::ConstSharedPtr grid_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr subscription_;
  double timeout_s_{5.0};
};
}  // namespace semantic_nav_costmap_layer
#endif
