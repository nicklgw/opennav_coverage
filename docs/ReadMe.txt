
https://github.com/Fields2Cover/Fields2Cover.git
使用代码标签tag:v1.2.1

将opennav_coverage的CMakeLists.txt中add_compile_options
将CMakeLists.txt编译选项-Wshadow去掉

min_turning_radius: 0.0
linear_curv_change: 1000000.0
设置这个参数,可以生成折线路径.

启动脚本
nick@nick-dell:~/opennav_ws$ ros2 launch opennav_coverage_demo coverage_demo_launch.py
