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

#include "nav2_rrt_planner/rrt_planner.hpp"

namespace nav2_rrt_planner
{
  RRT::RRT()
      : costmap_(nullptr),
        tf_(nullptr)
  {
  }

  void RRT::configure(
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
        node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".max_iter", rclcpp::ParameterValue(10000));
    node_->get_parameter(name_ + ".max_iter", _max_iter);
  }

  void RRT::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void RRT::activate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void RRT::deactivate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  nav_msgs::msg::Path RRT::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
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

    auto qos = rclcpp::QoS(1000);
    auto publisher = node_->create_publisher<visualization_msgs::msg::Marker>("/rrt", qos);
    publisher->on_activate();
    std_msgs::msg::ColorRGBA color;
    color.a = 1;
    color.r = 1;
    color.g = 0.5;
    color.b = 0;

    visualization_msgs::msg::Marker markerPoint;

    markerPoint.header.frame_id = global_frame_;
    markerPoint.header.stamp = node_->now();
    markerPoint.action = visualization_msgs::msg::Marker::MODIFY;
    markerPoint.type = visualization_msgs::msg::Marker::POINTS;
    markerPoint.id = 122323;
    markerPoint.scale.x = 0.05;
    markerPoint.scale.y = 0.05;

    markerPoint.color = color;

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_->now();
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.id = 2;
    marker.scale.x = 0.02;

    marker.color = color;

    // calculating the number of loops for current value of interpolation_resolution_

    std::vector<std::shared_ptr<nav2_rrt_planner::Node>> point_list;
    // add start to point_list
    auto start_node = std::make_shared<nav2_rrt_planner::Node>();
    start_node->x = start.pose.position.x;
    start_node->y = start.pose.position.y;
    start_node->parent = nullptr;
    point_list.push_back(start_node);
    RCLCPP_INFO(
        node_->get_logger(), "Starting path making. Start: X%fY%f | Goal: X%fY%f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    for (size_t i = 0; i < _max_iter; i++)
    {
      // generate random point/node
      unsigned int xm = (std::rand() % costmap_->getSizeInCellsX());
      unsigned int ym = (std::rand() % costmap_->getSizeInCellsY());
      double x, y;
      costmap_->mapToWorld(xm, ym, x, y);
      // RCLCPP_INFO(
      //     node_->get_logger(), "X %f, Y %f", x, y);
      // RCLCPP_INFO(
      //     node_->get_logger(), "X %f, Y %f", point_list[0]->x, point_list[0]->y);

      int nearest_neighbour = 0;
      double smallest_dist = std::numeric_limits<double>::max();
      // get nearest neighbour from point list
      for (size_t j = 0; j < point_list.size(); j++)
      {
        // distance check
        double dist = std::hypot(x - point_list[j]->x, y - point_list[j]->y);

        if (dist < smallest_dist)
        {
          smallest_dist = dist;
          nearest_neighbour = j;
        }
      }
      // RCLCPP_INFO(
      //     node_->get_logger(), "dist %f | %d", smallest_dist, nearest_neighbour);

      double delta_x = x - point_list[nearest_neighbour]->x;
      double delta_y = y - point_list[nearest_neighbour]->y;
      // clip node and add parent ^
      // auto angle = std::atan2(delta_x, delta_y);
      if (smallest_dist > interpolation_resolution_)
      {

        x = point_list[nearest_neighbour]->x + interpolation_resolution_ * delta_x / smallest_dist;
        y = point_list[nearest_neighbour]->y + interpolation_resolution_ * delta_y / smallest_dist;
      }
      unsigned int nearest_neighbour_xm, nearest_neighbour_ym;
      costmap_->worldToMap(x, y, xm, ym);
      costmap_->worldToMap(point_list[nearest_neighbour]->x, point_list[nearest_neighbour]->y, nearest_neighbour_xm, nearest_neighbour_ym);
      auto colliding = _collision_checker.lineCost(xm, nearest_neighbour_xm, ym, nearest_neighbour_ym) > MAX_NON_OBSTACLE;

      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = y;
      point.z = x;
      markerPoint.points.push_back(point);

      // return global_path;
      if (!colliding)
      {
        auto new_node = std::make_shared<nav2_rrt_planner::Node>();
        new_node->x = x;
        new_node->y = y;
        new_node->parent = point_list[nearest_neighbour].get();
        // RCLCPP_INFO(
        //     node_->get_logger(), "%f | %f", new_node.x, new_node.y);
        point_list.push_back(new_node);

        geometry_msgs::msg::Point p1;
        p1.x = x;
        p1.y = y;
        p1.z = goal.pose.position.z;
        geometry_msgs::msg::Point p2;
        p2.x = point_list[nearest_neighbour]->x;
        p2.y = point_list[nearest_neighbour]->y;
        // p2.x = start.pose.position.x;
        // p2.y = start.pose.position.y;
        p2.z = goal.pose.position.z;
        marker.points.push_back(p2);
        marker.points.push_back(p1);

        if (std::hypot(goal.pose.position.x - x, goal.pose.position.y - y) <= interpolation_resolution_)
        {
          publisher->publish(marker);
          publisher->publish(markerPoint);
          std::vector<geometry_msgs::msg::PoseStamped, std::allocator<geometry_msgs::msg::PoseStamped>> temp;
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position.x = goal.pose.position.x;
          pose.pose.position.y = goal.pose.position.y;
          pose.pose.position.z = 0.0;
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;
          pose.header.stamp = node_->now();
          pose.header.frame_id = global_frame_;
          temp.push_back(pose);
          nav2_rrt_planner::Node *curr = new_node.get();
          while (curr)
          {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = curr->x;
            pose.pose.position.y = curr->y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            curr = curr->parent;
            temp.push_back(pose);
          }

          for (int i = temp.size() - 1; i >= 0; i--)
          {
            RCLCPP_INFO(
                node_->get_logger(), "%f", temp[i].pose.position.x);
            global_path.poses.push_back(temp[i]);
          }

          RCLCPP_INFO(
              node_->get_logger(), "Found a path booya");
          return global_path;
        }
      }
      // if node not in collision(use collision checker)
      // then save point to point list
      // check if new node is close to goal
    }
    publisher->publish(marker);
    publisher->publish(markerPoint);

    RCLCPP_INFO(
        node_->get_logger(), "NO PATH !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    // int total_number_of_loop = std::hypot(
    //                                goal.pose.position.x - start.pose.position.x,
    //                                goal.pose.position.y - start.pose.position.y) /
    //                            interpolation_resolution_;
    // double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
    // double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

    // for (int i = 0; i < total_number_of_loop; ++i)
    // {
    //   geometry_msgs::msg::PoseStamped pose;
    //   pose.pose.position.x = start.pose.position.x + x_increment * i;
    //   pose.pose.position.y = start.pose.position.y + y_increment * i;
    //   pose.pose.position.z = 0.0;
    //   pose.pose.orientation.x = 0.0;
    //   pose.pose.orientation.y = 0.0;
    //   pose.pose.orientation.z = 0.0;
    //   pose.pose.orientation.w = 1.0;
    //   pose.header.stamp = node_->now();
    //   pose.header.frame_id = global_frame_;
    //   global_path.poses.push_back(pose);
    // }

    // geometry_msgs::msg::PoseStamped goal_pose = goal;
    // goal_pose.header.stamp = node_->now();
    // goal_pose.header.frame_id = global_frame_;
    // global_path.poses.push_back(goal_pose);

    return global_path;
  }

} // namespace nav2_rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrt_planner::RRT, nav2_core::GlobalPlanner)
