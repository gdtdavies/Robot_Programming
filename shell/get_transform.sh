cd ../
colcon build --packages-select cpp_nodes

source install/setup.sh

ros2 run cpp_nodes talker