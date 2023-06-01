//#include <behaviortree_ros2/bt_action_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>

#include <behaviortree_ros2/plugins.hpp>
#include "std_msgs/msg/string.hpp"
#include <filesystem>
#include <vector>
#include "../plugins/adding_service.cpp"
// #include "../plugins/service_plugin.cpp"

//#include "../plugins/listener_subscriber.hpp"
// #ifndef USE_SLEEP_PLUGIN
// #include "../plugins/sleep_action.hpp"
// #include "../plugins/listener_subscriber.hpp"
// #endif

using namespace BT;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sleep_client");

  BehaviorTreeFactory factory;
  static thread_local std::map<std::string, rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr> services;
  // std::vector<std::unique_ptr<BaseMK4Services>> humbleServices;
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

  // template<class ServiceT>
  // std::vector<std::shared_ptr<rclcpp::Client<ServiceT>>> service_clients;

  RosNodeParams params;
  params.nh = node;
  // params.default_port_value = "sleep_service";

  auto time_start = rclcpp::Clock().now();

  // RegisterRosNode(factory, "install/bt_ros_tests/lib/libsleep_plugin.so", params);
  //factory.registerNodeType<SleepAction>("Sleep", params);

  // params.default_port_value = "add_two_ints";
  factory.registerNodeType<AddTwoNumbers>("Adding", params, &services);
  // RegisterRosNode(factory, "install/bt_ros_tests/lib/libadding_plugin.so", params);

  // RegisterRosAction<AddTwoNumbers>(factory, params, 5);

  // params.default_port_value = "topic";
  // RegisterRosNode(factory, "install/bt_ros_tests/lib/libspeaking_plugin.so", params);
  // params.default_port_value = "topic";
  // RegisterRosNode(factory, "install/bt_ros_tests/lib/liblistening_plugin.so", params);

  std::string search_directory = "/home/eliot/Documents/poc_ws/src/PoCs/bt_ros_tests/trees/";

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

  // for (const auto& client : services)
  //   {
  //       const std::string& node_name = client.first;
  //       rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_ptr = client.second;

  //       // Find the node in the behavior tree and set the client
  //       if (auto custom_service_client_node = main_tree.rootNode()->findChild(node_name))
  //       {
  //           auto custom_service_client_wrapper = std::dynamic_pointer_cast<AddTwoNumbers>(custom_service_client_node);
  //           custom_service_client_wrapper->setClient(node, client_ptr);
  //       }
  //   }

  auto time_now = rclcpp::Clock().now();

  std::cout << "----- Loading took ----" << (time_now - time_start).seconds() << "seconds" << std::endl;
  //RCLCPP_INFO(this->get_logger(), "Loading took %f seconds", (time_now - time_start).seconds());

  main_tree.tickWhileRunning();

  //auto tree = factory.createTreeFromFile("/home/eliot/Documents/poc_ws/src/PoCs/bt_ros_tests/trees/main.xml");

  //tree.tickWhileRunning();

  return 0;
}

