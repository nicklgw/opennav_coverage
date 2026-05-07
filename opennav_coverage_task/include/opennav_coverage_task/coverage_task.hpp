
#ifndef OPENNAV_COVERAGE_TASK__COVERAGE_TASK_HPP_
#define OPENNAV_COVERAGE_TASK__COVERAGE_TASK_HPP_

#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "rics_navigation_behavior_msgs/action/move_through_poses.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "realtime_tools/realtime_buffer.hpp"
#include "semaphore.hpp"

namespace opennav_coverage_task
{

class CoverageTask : public nav2_util::LifecycleNode
{
public:

  explicit CoverageTask(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~CoverageTask() = default;

protected:
  /**
   * @brief Configure member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  rclcpp_action::Client<opennav_coverage_msgs::action::ComputeCoveragePath>::SharedPtr coverage_client_;
  rclcpp_action::Client<rics_navigation_behavior_msgs::action::MoveThroughPoses>::SharedPtr move_through_poses_client_;

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr field_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coverage_path_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gen_path_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr exe_path_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cnl_path_srv_; // 取消路径执行的服务，暂未实现

  void fieldPolygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  void genPathCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void exePathCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void cnlPathCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PolygonStamped>> field_polygon_;
  
  double nav_fixed_angle_{0.0};

  std::shared_ptr<std::thread> thread_;
  volatile bool do_stop_{false};
  int period_ms_{100};

  void run_();
  void stop_();

  details::Semaphore semaphore_;

  std::vector<opennav_coverage_msgs::msg::Swath> coverage_path_swaths_;

  std::atomic_bool gen_path_requested_{false};
  std::atomic_bool exe_path_requested_{false};
  std::atomic_bool cnl_path_requested_{false};

  void do_gen_path();
  void do_exe_path();
  void do_cnl_path();
};

}  // namespace opennav_coverage_task

#endif  // OPENNAV_COVERAGE_TASK__COVERAGE_TASK_HPP_
