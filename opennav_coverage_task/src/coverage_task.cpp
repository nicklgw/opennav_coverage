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
#include <nlohmann/json.hpp>

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

  declare_parameter("frequency", 20.0);
  double frequency = get_parameter("frequency").as_double();
  period_ms_ = 1000.0 / frequency;

  { // 初始化field_polygon_，避免在后续使用时出现空指针
    auto polygon_msg = std::make_shared<geometry_msgs::msg::PolygonStamped>();
    polygon_msg->polygon.points.push_back(geometry_msgs::msg::Point32());
    field_polygon_.writeFromNonRT(polygon_msg);
  }

  thread_ = std::shared_ptr<std::thread>(new std::thread(&CoverageTask::run_, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

void CoverageTask::run_()
{
  while (!do_stop_) 
  {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(period_ms_);
    
    if (gen_path_requested_)
    {
      do_gen_path();
      
      gen_path_requested_ = false;
    }
    
    if (exe_path_requested_)
    {
      do_exe_path();
      
      exe_path_requested_ = false;
    }

    semaphore_.waitUntil(end);
  }
}

void CoverageTask::stop_()
{
    do_stop_ = true;
    if (thread_.get()) {
        thread_->join();
        thread_.reset();
    }
}

void CoverageTask::do_gen_path()
{
  geometry_msgs::msg::PolygonStamped polygon_msg = **(field_polygon_.readFromRT());

  if (polygon_msg.polygon.points.size() != 4)
  {
    RCLCPP_ERROR(get_logger(), "reflector_polygon hasn't 4 vertexs.");
    return;
  }

  polygon_msg.header.stamp = this->now();
  polygon_msg.header.frame_id = "map";

#if 0
  polygon_msg.polygon.points.clear();

  geometry_msgs::msg::Point32 point;

  point.x = 0.0; point.y = 0.0; point.z = 0.0;
  polygon_msg.polygon.points.push_back(point);

  point.x = 0.0; point.y = 10.0; point.z = 0.0;
  polygon_msg.polygon.points.push_back(point);

  point.x = 15.0; point.y = 10.0; point.z = 0.0;
  polygon_msg.polygon.points.push_back(point);

  point.x = 15.0; point.y = 0.0; point.z = 0.0;
  polygon_msg.polygon.points.push_back(point);

  point.x = 0.0; point.y = 0.0; point.z = 0.0;
  polygon_msg.polygon.points.push_back(point);
#endif

  opennav_coverage_msgs::action::ComputeCoveragePath::Goal goal_;

  goal_.generate_headland = true;
  goal_.generate_route = true;
  goal_.generate_path = true;
  goal_.gml_field = "";
  goal_.use_gml_file = false;
  goal_.frame_id = "map";

  goal_.polygons.clear();
  goal_.polygons.resize(1);
  
  for (unsigned int j = 0; j != polygon_msg.polygon.points.size(); j++) 
  {
    opennav_coverage_msgs::msg::Coordinate coord;
    coord.axis1 = polygon_msg.polygon.points[j].x;
    coord.axis2 = polygon_msg.polygon.points[j].y;
    goal_.polygons[0].coordinates.push_back(coord);

    RCLCPP_INFO(get_logger(), "polygon.point %u, %.3f, %.3f", j, polygon_msg.polygon.points[j].x, polygon_msg.polygon.points[j].y);
  }
  
  opennav_coverage_msgs::msg::Coordinate coord;
  coord.axis1 = polygon_msg.polygon.points[0].x;
  coord.axis2 = polygon_msg.polygon.points[0].y;
  goal_.polygons[0].coordinates.push_back(coord);

  if (!coverage_client_->wait_for_action_server(5s)) 
  {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
  }

  auto send_goal_options = rclcpp_action::Client<opennav_coverage_msgs::action::ComputeCoveragePath>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<opennav_coverage_msgs::action::ComputeCoveragePath>> goal_handle)
    {
      if (!goal_handle) 
      {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      } 
      else 
      {
        RCLCPP_INFO(get_logger(), "Goal accepted by server");
      }
    };

  send_goal_options.feedback_callback =
    [this](rclcpp_action::ClientGoalHandle<opennav_coverage_msgs::action::ComputeCoveragePath>::SharedPtr,
            const std::shared_ptr<const opennav_coverage_msgs::action::ComputeCoveragePath::Feedback> feedback)
    {
      if (feedback) 
      {
        RCLCPP_INFO(get_logger(), "Received feedback");
      }
    };

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<opennav_coverage_msgs::action::ComputeCoveragePath>::WrappedResult & result)
    {
      switch (result.code) 
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        break;
      }

      // Print a brief summary if the action defines a result
      if (result.result) 
      {
        RCLCPP_INFO(get_logger(), "Result received error_code: %u ", result.result->error_code);

        coverage_path_swaths_ = result.result->coverage_path.swaths;
      }
    };

  coverage_client_->async_send_goal(goal_, send_goal_options);
}

void CoverageTask::do_exe_path()
{
  nlohmann::json root;
  root["orderId"] = std::string("my_id");
  nlohmann::json poses = nlohmann::json::array();

  for (unsigned int i = 0; i != coverage_path_swaths_.size(); i++)
  {
    auto & swath = coverage_path_swaths_[i];

    nlohmann::json pose;
    pose["x"] = swath.start.x;
    pose["y"] = swath.start.y;
    pose["angle"] = 0.0;
    poses.push_back(pose);

    pose["x"] = swath.end.x;
    pose["y"] = swath.end.y;
    pose["angle"] = 0.0;    
    poses.push_back(pose);
  }

  root["poses"] = poses;

  std::string json_poses = root.dump();
  RCLCPP_INFO(get_logger(), "Generated JSON: %s", json_poses.c_str());

  rics_navigation_behavior_msgs::action::MoveThroughPoses::Goal goal_;
  goal_.json_poses = json_poses;

  if (!move_through_poses_client_->wait_for_action_server(5s)) 
  {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
  }
 
  auto send_goal_options = rclcpp_action::Client<rics_navigation_behavior_msgs::action::MoveThroughPoses>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<rics_navigation_behavior_msgs::action::MoveThroughPoses>> goal_handle)
    {
      if (!goal_handle) 
      {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      } 
      else 
      {
        RCLCPP_INFO(get_logger(), "Goal accepted by server");
      }
    };

  send_goal_options.feedback_callback =
    [this](rclcpp_action::ClientGoalHandle<rics_navigation_behavior_msgs::action::MoveThroughPoses>::SharedPtr,
            const std::shared_ptr<const rics_navigation_behavior_msgs::action::MoveThroughPoses::Feedback> feedback)
    {
      if (feedback) 
      {
        RCLCPP_INFO(get_logger(), "Received feedback");
        // feedback->current_node_id;
        // feedback->goal_node_id;
        // feedback->current_pose;
        // feedback->navigation_time;
        // feedback->estimated_time_remaining;
        // feedback->number_of_recoveries;
        // feedback->distance_remaining;
        // feedback->number_of_poses_remaining;
      }
    };

  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<rics_navigation_behavior_msgs::action::MoveThroughPoses>::WrappedResult & result)
    {
      switch (result.code) 
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        break;
      }

      // Print a brief summary if the action defines a result
      if (result.result) 
      {
        RCLCPP_INFO(get_logger(), "Result received error_code: %s ", result.result->result_code.c_str());
      }
    };
  
//  move_through_poses_client_->async_send_goal(goal_, send_goal_options);

  // auto cancel_future = move_through_poses_client_->async_cancel_all_goals();
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
  move_through_poses_client_ = rclcpp_action::create_client<rics_navigation_behavior_msgs::action::MoveThroughPoses>(this, "/rics_behavior/NavigationBehavior/MoveThroughPoses");
  
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
  
  gen_path_requested_ = true;

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

  exe_path_requested_  = true;

  response->success = true;
  
  RCLCPP_INFO(get_logger(), "Received request to execute coverage path");
}

nav2_util::CallbackReturn
CoverageTask::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  stop_();

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
