#include <string>
#include "talker_publisher.hpp"
#include <behaviortree_ros2/plugins.hpp>

bool CountPublish::setMessage(std_msgs::msg::String& msg)
{
  auto message = getInput<std::string>("sentence").value();
  msg.data = message;
  return true;
}

CreateRosNodePlugin(CountPublish, "Speak");
