source ../../limo_ros2/install/setup.sh
source /opt/ros/humble/setup.sh

ros2 launch limo_navigation limo_navigation.launch.py use_sim_time:=true map:=../maps/assignment_map2.yaml params_file:=../params/nav2_params.yaml