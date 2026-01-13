cd ../
colcon build
source install/setup.bash
ros2 launch grpc_commander grpc_client.launch.py
