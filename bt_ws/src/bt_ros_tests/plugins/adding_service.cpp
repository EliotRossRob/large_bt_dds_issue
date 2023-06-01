#include <string>
#include <vector>
#include <behaviortree_ros2/bt_service_node.hpp>
#include "example_interfaces/srv/add_two_ints.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_ros2/ros_node_params.hpp"
// #include "behaviortree_ros2/service_clients_list.hpp"

using namespace BT;

class AddTwoNumbers: public RosServiceNode<example_interfaces::srv::AddTwoInts>
{
public:
  AddTwoNumbers(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params,
              std::map<std::string, rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr> * arg_vect)
    : RosServiceNode<example_interfaces::srv::AddTwoInts>(name, conf, params, arg_vect)
  {
    // int test = arg_int;
    // RCLCPP_INFO( node_->get_logger(), "test number: %d", test);
  }

  static BT::PortsList providedPorts()
  {
    PortsList addition;
    PortsList basic = {
      InputPort<int>("number1"),
      InputPort<int>("number2"),
      OutputPort<std::string>("sum")
    };
    basic.insert(addition.begin(), addition.end());

    return providedBasicPorts({basic});
  }

  bool setRequest(RosServiceNode::Request::SharedPtr& request)
  {
    auto number1 = getInput<int>("number1");
    auto number2 = getInput<int>("number2");
    request->a = number1.value();
    request->b = number2.value();
    return true;
  }

  NodeStatus onResponseReceived(const typename RosServiceNode::Response::SharedPtr& response)
  {
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived %ld", name().c_str(), response->sum );
    std::string sum = std::to_string(response->sum);
    setOutput("sum", sum);
    return NodeStatus::SUCCESS;
  }

  NodeStatus onFailure(ServiceNodeErrorCode error)
  {
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure %d", name().c_str(), error );
    return NodeStatus::FAILURE;
  }

};

// BT_REGISTER_NODES(factory) {
//   BT::RosNodeParams params;
//   factory.registerNodeType<AddTwoNumbers>("Adding", params, );
// }
