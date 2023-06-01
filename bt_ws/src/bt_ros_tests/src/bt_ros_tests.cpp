
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>

#include <behaviortree_ros2/plugins.hpp>
#include "std_msgs/msg/string.hpp"
#include <filesystem>
#include <vector>
#include "../plugins/adding_service.cpp"


using namespace BT;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sleep_client");

  BehaviorTreeFactory factory;
  static thread_local std::map<std::string, rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr> services;

  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::string service_name = "add_two_ints";
  rclcpp::CallbackGroup::SharedPtr callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());
  auto service = node->create_client<example_interfaces::srv::AddTwoInts>(service_name, rmw_qos_profile_services_default, callback_group_);
  services[service_name] = service;

  bool found = service->wait_for_service();
  if(!found)
  {
    RCLCPP_ERROR(node->get_logger(), "Service is not reachable");
  }

  RosNodeParams params;
  params.nh = node;

  auto time_start = rclcpp::Clock().now();

  factory.registerNodeType<AddTwoNumbers>("Adding", params, &services);

  std::string search_directory = "/home/eliot/Documents/large_bt_dds_issue/bt_ws/src/bt_ros_tests/trees/";

  using std::filesystem::directory_iterator;
  for (auto const& entry : directory_iterator(search_directory))
  {
    if( entry.path().extension() == ".xml")
    {
      factory.registerBehaviorTreeFromFile(entry.path().string());
    }
  }

  std::cout << "----- MainTree tick ----" << std::endl;
  auto main_tree = factory.createTree("MainTree");

  auto time_now = rclcpp::Clock().now();

  std::cout << "----- Loading took ----" << (time_now - time_start).seconds() << "seconds" << std::endl;

  main_tree.tickWhileRunning();

  return 0;
}

