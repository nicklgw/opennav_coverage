
启动脚本
orangepi@orangepi5plus:~/opennav_ws$ ros2 launch opennav_coverage_task coverage_task.launch.py


生成全覆盖路径
nick@nick-dell:~$ ros2 service call /coverage_task/gen_path std_srvs/srv/Trigger '{}'
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='')

nick@nick-dell:~$ ros2 service call /coverage_task/gen_path std_srvs/srv/Trigger '{}'
requester: making request: std_srvs.srv.Trigger_Request()

response:
std_srvs.srv.Trigger_Response(success=True, message='')

