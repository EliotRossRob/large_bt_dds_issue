cmake_minimum_required(VERSION 3.16)
project(bt_ros_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp
    behaviortree_ros2
    example_interfaces
    std_msgs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED )
find_package(rclcpp_action REQUIRED )
find_package(example_interfaces REQUIRED )
find_package(std_msgs REQUIRED )
find_package(behaviortree_cpp REQUIRED )
find_package(behaviortree_ros2 REQUIRED )
find_package(ament_index_cpp REQUIRED)

# macro to remove some boiler plate
function(add_target_dependencies target)
    target_include_directories(${target} PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
    ament_target_dependencies(${target} ${THIS_PACKAGE_INCLUDE_DEPENDS})

endfunction()

######################################################
# Compile Sleep library as static dependeny or plugin

add_library(sleep_plugin SHARED plugins/sleep_action.cpp)
target_compile_definitions(sleep_plugin PRIVATE  BT_PLUGIN_EXPORT )
add_target_dependencies(sleep_plugin)

add_library(sleep_static STATIC plugins/sleep_action.cpp)
add_target_dependencies(sleep_static)

######################################################
# Compile Adding library as static dependeny or plugin

add_library(adding_plugin SHARED plugins/adding_service.cpp)
target_compile_definitions(adding_plugin PRIVATE  BT_PLUGIN_EXPORT )
add_target_dependencies(adding_plugin)

add_library(adding_static STATIC plugins/adding_service.cpp)
add_target_dependencies(adding_static)

######################################################
# Compile Speaking library as static dependeny or plugin

add_library(speaking_plugin SHARED plugins/talker_publisher.cpp)
target_compile_definitions(speaking_plugin PRIVATE  BT_PLUGIN_EXPORT )
add_target_dependencies(speaking_plugin)

add_library(speaking_static STATIC plugins/talker_publisher.cpp)
add_target_dependencies(speaking_static)

######################################################
# Compile Listening library as static dependeny or plugin

add_library(listening_plugin SHARED plugins/listener_subscriber.cpp)
target_compile_definitions(listening_plugin PRIVATE  BT_PLUGIN_EXPORT )
add_target_dependencies(listening_plugin)

add_library(listening_static STATIC plugins/listener_subscriber.cpp)
add_target_dependencies(listening_static)

#####################################################
# Compile Service library as static dependeny or plugin

# add_library(service_plugin SHARED plugins/service_plugin.cpp)
# target_compile_definitions(service_plugin PRIVATE  BT_PLUGIN_EXPORT )
# add_target_dependencies(service_plugin)

# add_library(service_static STATIC plugins/service_plugin.cpp)
# add_target_dependencies(service_static)


######################################################
# Build two clients, one that use the plugin and the other
# that links statically

add_executable(bt_ros_tests_dyn src/bt_ros_tests.cpp)
target_compile_definitions(bt_ros_tests_dyn PRIVATE USE_SLEEP_PLUGIN )
add_target_dependencies(bt_ros_tests_dyn)

add_executable(bt_ros_tests src/bt_ros_tests.cpp)
target_link_libraries(bt_ros_tests sleep_static )
add_target_dependencies(bt_ros_tests)

######################################################
# Build Server
add_executable(sleep_server src/sleep_server.cpp)
add_target_dependencies(sleep_server)

add_executable(adding_server src/add_two_ints.cpp)
add_target_dependencies(adding_server)

add_executable(adding_server_pt2 src/add_two_ints_pt2.cpp)
add_target_dependencies(adding_server_pt2)

add_executable(speaking_server src/publisher_talker.cpp)
add_target_dependencies(speaking_server)

add_executable(listening_server src/subscriber_listener.cpp)
add_target_dependencies(listening_server)
######################################################

ament_target_dependencies(
  ${PROJECT_NAME}
  behaviortree_cpp
  behaviortree_ros2
)

# INSTALL

install(TARGETS
  bt_ros_tests
  bt_ros_tests_dyn
  sleep_server
  adding_server
  adding_server_pt2
  speaking_server
  listening_server
  sleep_plugin
  listening_plugin
  speaking_plugin
  adding_plugin
  # service_plugin
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  )

  install( TARGETS
    ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY include trees plugins launch
  DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS bt_ros_tests sleep_server adding_server adding_server_pt2 speaking_server listening_server DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(behaviortree_cpp behaviortree_ros2 std_msgs rclcpp_action rclcpp)

ament_package()