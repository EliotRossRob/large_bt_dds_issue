#include <string>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include "std_msgs/msg/string.hpp"

using namespace BT;

class ListenerSub: public RosTopicSubNode<std_msgs::msg::String>
{
public:
  ListenerSub(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosTopicSubNode<std_msgs::msg::String>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({OutputPort<std::string>("sum")});
  }

  void topicCallback(const std::shared_ptr<std_msgs::msg::String> msg) override;

  BT::NodeStatus onTick(const typename std_msgs::msg::String::SharedPtr& last_msg) override;

};