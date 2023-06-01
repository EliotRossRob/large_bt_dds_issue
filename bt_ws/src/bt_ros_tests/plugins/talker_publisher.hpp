#include <string>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include "std_msgs/msg/string.hpp"

using namespace BT;

class CountPublish: public RosTopicPubNode<std_msgs::msg::String>
{
public:
  CountPublish(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosTopicPubNode<std_msgs::msg::String>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("sentence")});
  }

  bool setMessage(std_msgs::msg::String& msg) override;

};