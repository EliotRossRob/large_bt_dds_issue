#ifndef BEHAVIORTREE_ROS2__SERVICE_CLIENTS_LIST_HPP_
#define BEHAVIORTREE_ROS2__SERVICE_CLIENTS_LIST_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <map>

namespace BT
{

template<class ServiceT>
struct ServiceClients
{
  std::map<std::string, std::shared_ptr<rclcpp::Client<ServiceT>>> service_clients;
};
}

#endif // BEHAVIORTREE_ROS2__SERVICE_CLIENTS_LIST_HPP_
