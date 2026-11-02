sudo ip addr flush dev enP8p1s0
sudo ip addr add 192.168.1.5/24 dev enP8p1s0
sudo ip link set enP8p1s0 up

cd ~/livox_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

cd ~/lio_ws
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml


cd ~/scan_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scan_planner scan_planner.launch.py use_rviz:=false \
  cloud_topic:=/cloud_registered \
  odom_topic:=/Odometry \
  global_path_topic:=/scan_global_path \
  planning_frame:=camera_init

ros2 topic pub --once /trigger_plan std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /trigger_plan std_msgs/msg/Bool "{data: false}"


cd ~/scan_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scan_cmd_vel_follower scan_cmd_vel_follower.launch.py \
  path_topic:=/visual_local_trajectory \
  odom_topic:=/Odometry \
  cmd_topic:=/scan_cmd_vel \
  planning_frame:=camera_init \
  max_linear_x:=0.005 \
  max_angular_z:=0.02 \
  odom_timeout:=2.0 \
  path_timeout:=2.0


source /opt/ros/humble/setup.bash
source ~/atom01_deploy/install/setup.bash
ros2 topic echo /scan_cmd_vel_debug geometry_msgs/msg/Twist --qos-reliability best_effort

ros2 topic info -v /scan_cmd_vel_debug



cd ~/atom01_deploy
./scripts/can.sh
./tools/start_robot.sh

cd ~/scan_ws
bash tools/start_upper_stack.sh --configure-link
tail -f /tmp/scan_upper_stack/*.log

bash ~/scan_ws/tools/trigger_scan_plan.sh start

bash ~/scan_ws/tools/trigger_scan_plan.sh stop


bash ~/scan_ws/tools/stop_upper_stack.sh


cd ~/atom01_deploy
bash tools/start_lower_stack.sh

tail -f /tmp/atom_lower_stack/*.log

bash ~/atom01_deploy/tools/stop_lower_stack.sh


pkill -f scan_cmd_vel_follower
pkill -f scan_planner
pkill -f fast_lio
pkill -f livox_ros_driver2
ros2 daemon stop


ros2 topic echo /visual_local_trajectory --once
ros2 topic echo /cmd_vel --once --qos-reliability best_effort


screen -S inference_session -X quit
screen -S joy_session -X quit
pkill -f inference
pkill -f joy_node
ros2 daemon stop



ros2 topic echo /action --once --qos-reliability best_effort
ros2 topic echo /joint_states --once --qos-reliability best_effort

ros2 topic info -v /cmd_vel --no-daemon


screen -r inference_session

ros2 topic echo /Odometry --once --qos-reliability best_effort