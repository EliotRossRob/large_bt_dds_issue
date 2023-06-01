#ifndef BT_ROS_TESTS__COMMON__SERVICE_CLIENTS_HPP_
#define BT_ROS_TESTS__COMMON__SERVICE_CLIENTS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <map>

namespace BT
{

template<class ServiceT>
struct Clients
{
  std::map<std::string, std::shared_ptr<rclcpp::Client<ServiceT>>> service_clients;
};
}

#endif // BT_ROS_TESTS__COMMON__SERVICE_CLIENTS_HPP_