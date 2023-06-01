# large_bt_dds_issue

# Some background

I am using Ros2 humble which is not fully supported by the main BT.cpp and BT.ROS as they are for older versions, this is the reason I have taken from branches of them both and brought them into the repository.

The only folders that you really need to be concerned with are in `bt_ros_tests` and `BehaviorTree.ROS2-humble` files, the BT.cpp should not be touched

Within those files I am only really working with the services for now, so inside `BehaviorTree.ROS2-humble/include/behaviortree_ros2` is the `bt_service_node.hpp`. This could potentially be the place for error, as well as in `bt_ros_tests/src/bt_ros_test.cpp`

I have a feeling the issue is with how I am passing my clients through into the service node however I haven't been able to confirm that.

apologies if this code is quite alot of filler.

# Build

I am using colcon build to build my ros2 packages, `colcon build --packages-select behaviortree_ros2 behaviortree_cpp bt_ros_tests --symlink-install`
You may need to install some dependencies such as the example_interfaces and std_msgs if not already.

# To run

this will need two terminals

- one to launch the servers `ros2 launch bt_ros_tests bt_drivers.launch.py`
- one to run the bt tree `ros2 run bt_ros_tests bt_ros_tests`
- you will need to change the path location for you main_tree inside `bt_ros_test.cpp`
- The main tree is found in the trees folder named main.xml

# Useful links

If you need to have a look at what behavior trees do:

https://www.behaviortree.dev/docs/Intro


