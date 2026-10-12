// Copyright 2026 semantic-nav-ros contributors. SPDX-License-Identifier: Apache-2.0
#include "semantic_nav_costmap_layer/semantic_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace semantic_nav_costmap_layer
{
void SemanticLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("SemanticLayer lifecycle node expired");}
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/semantic_costmap")));
  declareParameter("observation_timeout", rclcpp::ParameterValue(5.0));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".observation_timeout", timeout_s_);
  if (!std::isfinite(timeout_s_) || timeout_s_ <= 0.0) {
    throw std::invalid_argument("observation_timeout must be finite and positive");
  }
  std::string topic;
  node->get_parameter(name_ + ".topic", topic);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](nav2_msgs::msg::Costmap::ConstSharedPtr grid) {receiveGrid(grid);}, options);
  current_ = true;
  RCLCPP_INFO(logger_, "Semantic layer listening on %s; latest TF; timeout %.2f sim seconds",
    topic.c_str(), timeout_s_);
}

void SemanticLayer::receiveGrid(nav2_msgs::msg::Costmap::ConstSharedPtr grid)
{
  const auto & m = grid->metadata;
  const auto & p = m.origin.position;
  const auto & q = m.origin.orientation;
  // Planar maps only. Reject malformed observations without refreshing their age.
  if (grid->header.frame_id.empty() || m.size_x == 0 || m.size_y == 0 ||
    !std::isfinite(m.resolution) || m.resolution <= 0.0 ||
    static_cast<uint64_t>(m.size_x) * m.size_y != grid->data.size() ||
    !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
    std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 ||
    std::abs(q.z * q.z + q.w * q.w - 1.0) > 1e-3)
  {
    RCLCPP_WARN(logger_, "Rejected malformed/nonplanar semantic grid");
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  grid_ = std::move(grid);
}

void SemanticLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  grid_.reset();
  current_ = true;
}

void SemanticLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  // Full redraw is deliberate for this small arena: it clears replaced/expired
  // observations and follows rolling windows without retaining ghost costs.
  auto map = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, map->getOriginX());
  *min_y = std::min(*min_y, map->getOriginY());
  *max_x = std::max(*max_x, map->getOriginX() + map->getSizeInMetersX());
  *max_y = std::max(*max_y, map->getOriginY() + map->getSizeInMetersY());
}

void SemanticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}
  nav2_msgs::msg::Costmap::ConstSharedPtr grid;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid = grid_;
  }
  if (!grid) {return;}
  const double age = (clock_->now() - rclcpp::Time(grid->header.stamp, clock_->get_clock_type())).seconds();
  // Source stamps are simulated observation time, including cloud inference latency.
  if (age < 0.0 || age > timeout_s_) {return;}
  tf2::Transform source_from_master;
  source_from_master.setIdentity();
  if (grid->header.frame_id != layered_costmap_->getGlobalFrameID()) {
    try {
      const auto transform = tf_->lookupTransform(
        grid->header.frame_id, layered_costmap_->getGlobalFrameID(), tf2::TimePointZero);
      tf2::fromMsg(transform.transform, source_from_master);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Semantic grid TF unavailable: %s", ex.what());
      return;
    }
  }
  tf2::Transform source_from_grid;
  tf2::fromMsg(grid->metadata.origin, source_from_grid);
  const auto grid_from_master = source_from_grid.inverse() * source_from_master;
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(master.getSizeInCellsX()), max_i);
  max_j = std::min(static_cast<int>(master.getSizeInCellsY()), max_j);
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master.mapToWorld(i, j, wx, wy);
      const auto point = grid_from_master * tf2::Vector3(wx, wy, 0.0);
      const double gx = std::floor(point.x() / grid->metadata.resolution);
      const double gy = std::floor(point.y() / grid->metadata.resolution);
      if (gx < 0 || gy < 0 || gx >= grid->metadata.size_x || gy >= grid->metadata.size_y) {
        continue;
      }
      const auto cost = grid->data[static_cast<size_t>(gy) * grid->metadata.size_x +
        static_cast<size_t>(gx)];
      // Zero is no added penalty, 255 is no semantic observation. Neither clears
      // lidar/static obstacles or converts unknown space to known free space.
      if (cost == 0 || cost == nav2_costmap_2d::NO_INFORMATION) {continue;}
      const auto old = master.getCost(i, j);
      if (old == nav2_costmap_2d::NO_INFORMATION || cost > old) {
        master.setCost(i, j, cost);
      }
    }
  }
}
}  // namespace semantic_nav_costmap_layer
PLUGINLIB_EXPORT_CLASS(semantic_nav_costmap_layer::SemanticLayer, nav2_costmap_2d::Layer)
