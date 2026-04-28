// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "opennav_coverage_task/coverage_task.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;

namespace opennav_coverage_task
{

CoverageTask::CoverageTask(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("coverage_task", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());
}

nav2_util::CallbackReturn
CoverageTask::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  { // 初始化field_polygon_，避免在后续使用时出现空指针
    auto polygon_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>();
    polygon_msg->polygon.points.push_back(geometry_msgs::msg::Point32());
    field_polygon_.writeFromNonRT(polygon_msg);
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageTask::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());
  auto node = shared_from_this();

  field_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/reflector_polygon", rclcpp::QoS(1),
    std::bind(&CoverageTask::fieldPolygonCallback, this, std::placeholders::_1));

  coverage_path_pub_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
    this,
    "coverage_task/coverage_path", rclcpp::QoS(1));

  coverage_client_ = rclcpp_action::create_client<opennav_coverage_msgs::action::ComputeCoveragePath>(this, "compute_coverage_path");
  through_poses_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");
  
  gen_path_srv_ = node->create_service<std_srvs::srv::Trigger>(
    std::string("coverage_task/gen_path"),
    std::bind(
      &CoverageTask::genPathCb, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  
  exe_path_srv_ = node->create_service<std_srvs::srv::Trigger>(
    std::string("coverage_task/exe_path"),
    std::bind(
      &CoverageTask::exePathCb, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  
  // create bond connection
  createBond();
  
  return nav2_util::CallbackReturn::SUCCESS;
}

void CoverageTask::fieldPolygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  field_polygon_.writeFromNonRT(msg);
}

void CoverageTask::genPathCb(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;
  response->success = true;

  RCLCPP_INFO(get_logger(), "Received request to generate coverage path");
}

void CoverageTask::exePathCb(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;
  response->success = true;
  
  RCLCPP_INFO(get_logger(), "Received request to execute coverage path");
}

nav2_util::CallbackReturn
CoverageTask::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageTask::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CoverageTask::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace opennav_coverage_task

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_coverage_task::CoverageTask)
