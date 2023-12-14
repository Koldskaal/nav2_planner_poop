/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "nav2_prm_planner/prm_planner.hpp"

namespace nav2_prm_planner
{
  void PRM::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    _collision_checker = nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>(costmap_);
    _collision_checker.setCostmap(costmap_);
    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".max_edge_length", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".max_edge_length", _max_edge_length);
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".max_iter", rclcpp::ParameterValue(10000));
    node_->get_parameter(name_ + ".max_iter", _max_iter);
  }

  void PRM::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void PRM::activate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void PRM::deactivate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  nav_msgs::msg::Path PRM::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    auto qos = rclcpp::QoS(5000);
    auto publisher = node_->create_publisher<visualization_msgs::msg::Marker>("/prm", qos);
    publisher->on_activate();

    if (nodes.size() == 0)
    {

      std_msgs::msg::ColorRGBA color;
      color.a = 1;
      color.r = 0;
      color.g = 0.5;
      color.b = 1;

      visualization_msgs::msg::Marker markerPoint;

      markerPoint.header.frame_id = global_frame_;
      markerPoint.header.stamp = node_->now();
      markerPoint.action = visualization_msgs::msg::Marker::ADD;
      markerPoint.type = visualization_msgs::msg::Marker::POINTS;
      markerPoint.id = 1;
      markerPoint.scale.x = 0.05;
      markerPoint.scale.y = 0.05;

      markerPoint.color = color;
      visualization_msgs::msg::Marker lineMarker;

      lineMarker.header.frame_id = global_frame_;
      lineMarker.header.stamp = node_->now();
      lineMarker.action = visualization_msgs::msg::Marker::MODIFY;
      lineMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
      lineMarker.id = 2;
      lineMarker.scale.x = 0.01;
      lineMarker.color = color;

      // Make the roadmap
      for (size_t i = 0; i < _max_iter; i++)
      {
        unsigned int xm = std::rand() % costmap_->getSizeInCellsX();
        unsigned int ym = std::rand() % costmap_->getSizeInCellsY();
        if (costmap_->getCost(xm, ym) > MAX_NON_OBSTACLE)
          continue;

        nav2_prm_planner::Point newPoint = {xm, ym};
        nodes.push_back(newPoint);

        double x, y;
        costmap_->mapToWorld(xm, ym, x, y);
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = x;
        markerPoint.points.push_back(point);
      }

      publisher->publish(markerPoint);
      edges = std::vector<std::vector<nav2_prm_planner::Edge>>((int)nodes.size());

      for (size_t i = 0; i < nodes.size(); i++)
      {
        // do connection search

        for (unsigned int j = 0; j < nodes.size(); j++)
        {
          if (j == i)
            continue;
          auto p1 = nodes[i];
          auto p2 = nodes[j];
          float dist = std::hypot(p2.x - p1.x, p2.y - p1.y);

          if (dist > _max_edge_length)
          {
            continue;
          }

          auto colliding = _collision_checker.lineCost(p1.x, p2.x, p1.y, p2.y) > MAX_NON_OBSTACLE;
          if (colliding)
          {
            continue;
          }
          Edge newEdge = {j, dist};
          edges[i].push_back(newEdge);

          geometry_msgs::msg::Point pp1;
          double x1, x2, y1, y2;
          costmap_->mapToWorld(p1.x, p1.y, x1, y1);
          costmap_->mapToWorld(p2.x, p2.y, x2, y2);
          pp1.x = x1;
          pp1.y = y1;
          pp1.z = goal.pose.position.z;
          geometry_msgs::msg::Point pp2;
          pp2.x = x2;
          pp2.y = y2;
          pp2.z = goal.pose.position.z;
          lineMarker.points.push_back(pp2);
          lineMarker.points.push_back(pp1);
        }
      }

      publisher->publish(lineMarker);
    }

    nav_msgs::msg::Path global_path;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "Planner will only except start position from %s frame",
          global_frame_.c_str());
      return global_path;
    }

    if (goal.header.frame_id != global_frame_)
    {
      RCLCPP_INFO(
          node_->get_logger(), "Planner will only except goal position from %s frame",
          global_frame_.c_str());
      return global_path;
    }
    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    for (size_t i = 0; i < nodes.size(); i++)
    {
    }

    // calculating the number of loops for current value of interpolation_resolution_

    // implent dijstrka

    return global_path;
  }

} // namespace nav2_rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_prm_planner::PRM, nav2_core::GlobalPlanner)
