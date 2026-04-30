
启动脚本
orangepi@orangepi5plus:~/opennav_ws$ ros2 launch opennav_coverage_task coverage_task.launch.py


生成全覆盖路径
nick@nick-dell:~$ ros2 service call /coverage_task/gen_path std_srvs/srv/Trigger '{}'
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='')

执行全覆盖路径
nick@nick-dell:~$ ros2 service call /coverage_task/exe_path std_srvs/srv/Trigger '{}'
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='')



/rics_behavior/NavigationBehavior/MoveThroughPoses: rics_navigation_behavior_msgs/action/MoveThroughPoses


orangepi@orangepi5plus:~$ ros2 node info /NavigationBehavior
/NavigationBehavior
  Subscribers:
    /NavigationBehavior/command: rics_service_msgs/msg/ServiceControl
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rics/rics_event: rics_main_controller_msgs/msg/RicsEvent
  Publishers:
    /NavigationBehavior/transition_event: lifecycle_msgs/msg/TransitionEvent
    /diagnostics: diagnostic_msgs/msg/DiagnosticArray
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rics_behavior/action_status_changed: rics_task_msgs/msg/BehaviorActionStatusChanged
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /NavigationBehavior/change_state: lifecycle_msgs/srv/ChangeState
    /NavigationBehavior/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /NavigationBehavior/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /NavigationBehavior/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /NavigationBehavior/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /NavigationBehavior/get_parameters: rcl_interfaces/srv/GetParameters
    /NavigationBehavior/get_state: lifecycle_msgs/srv/GetState
    /NavigationBehavior/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /NavigationBehavior/list_parameters: rcl_interfaces/srv/ListParameters
    /NavigationBehavior/navigation_params: rics_navigation_behavior_msgs/srv/NavParameters
    /NavigationBehavior/set_parameters: rcl_interfaces/srv/SetParameters
    /NavigationBehavior/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /NavigationBehavior/command: rics_service_msgs/action/ServiceControl
    /rics_behavior/NavigationBehavior/GetCurrentPose: rics_navigation_behavior_msgs/action/GetCurrentPose
    /rics_behavior/NavigationBehavior/LimitSpeed: rics_navigation_behavior_msgs/action/LimitSpeed
    /rics_behavior/NavigationBehavior/MoveDistance: rics_navigation_behavior_msgs/action/MoveDistance
    /rics_behavior/NavigationBehavior/MoveThroughPoses: rics_navigation_behavior_msgs/action/MoveThroughPoses
    /rics_behavior/NavigationBehavior/MoveTo: rics_navigation_behavior_msgs/action/MoveTo
    /rics_behavior/NavigationBehavior/SetNavParams: rics_navigation_behavior_msgs/action/SetNavParams
    /rics_behavior/NavigationBehavior/StopMoving: rics_navigation_behavior_msgs/action/StopMoving
  Action Clients:

orangepi@orangepi5plus:~$

orangepi@orangepi5plus:~$ ros2 interface show rics_navigation_behavior_msgs/action/MoveThroughPoses
string behavior_tree
string json_poses
rcl_interfaces/Parameter[] params
        string name
        ParameterValue value
                uint8 type
                bool bool_value
                int64 integer_value
                float64 double_value
                string string_value
                byte[] byte_array_value
                bool[] bool_array_value
                int64[] integer_array_value
                float64[] double_array_value
                string[] string_array_value
---
#result definition
string result_code
---
#feedback definition
string current_node_id
string goal_node_id
geometry_msgs/PoseStamped current_pose
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        Pose pose
                Point position
                        float64 x
                        float64 y
                        float64 z
                Quaternion orientation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1
builtin_interfaces/Duration navigation_time
        int32 sec
        uint32 nanosec
builtin_interfaces/Duration estimated_time_remaining
        int32 sec
        uint32 nanosec
int16 number_of_recoveries
float32 distance_remaining
int16 number_of_poses_remaining
orangepi@orangepi5plus:~$
