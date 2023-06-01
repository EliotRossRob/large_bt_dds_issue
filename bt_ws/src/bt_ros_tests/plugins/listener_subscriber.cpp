#include <string>
#include "listener_subscriber.hpp"
#include "std_msgs/msg/string.hpp"
#include <behaviortree_ros2/plugins.hpp>

void ListenerSub::topicCallback(const std::shared_ptr<std_msgs::msg::String> msg)
{
  //auto message = getInput<std::string>("sentence").value();
  RCLCPP_INFO( node_->get_logger(), "%s: Topic Callback 1 %s", name().c_str(), msg->data.c_str() );
  setOutput("sum", msg->data.c_str());
}

BT::NodeStatus ListenerSub::onTick(const typename std_msgs::msg::String::SharedPtr& last_msg)
{
  //setOutput("sum", last_msg->data.c_str());
  //RCLCPP_INFO( node_->get_logger(), "%s: Topic Callback 2 %s", name().c_str(), last_msg->data.c_str() );
  int i = 0;
  i = i + 1;
  return NodeStatus::SUCCESS;
}

CreateRosNodePlugin(ListenerSub, "Listen");
