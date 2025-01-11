//from auto_explore_mapper map_server.cpp

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include <slam_toolbox/srv/detail/save_map__struct.hpp>
#include "nav2_msgs/srv/save_map.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
using namespace slam_toolbox;
using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  /*****
   * use slam_toolbox
   */
  // std::shared_ptr<rclcpp::Node> map_serializer_node = rclcpp::Node::make_shared("map_serializer_client");
  // rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr map_serializer_client =
  //   map_serializer_node->create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");

  // auto serialize_pose_graph_request = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();

  // while (!map_serializer_client->wait_for_service(2s)) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the map serializer service. Exiting.");
  //     return 0;
  //   }
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Map serializer service not available, waiting again...");
  // }

  // serialize_pose_graph_request->filename = "~/AutoExploreMap";
  // auto result = map_serializer_client->async_send_request(serialize_pose_graph_request);

  //////Need to launch slam_toolbox_node: ros2 run slam_toolbox sync_slam_toolbox_node
  // std::shared_ptr<rclcpp::Node> map_saver_node = rclcpp::Node::make_shared("map_saver_client");
  // rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr map_saver_client =
  //   map_saver_node->create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");

  // auto save_map_request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();

  // while (!map_saver_client->wait_for_service(2s)) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the SaveMap service. Exiting.");
  //     return 0;
  //   }
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SaveMap service not available, waiting again...");
  // }

  // save_map_request->name.data = "~/AutoExploreMap";
  // auto map_saver_result = map_saver_client->async_send_request(save_map_request);

  // // Wait for the result.
  // if (rclcpp::spin_until_future_complete(map_saver_node, map_saver_result) ==
  //   rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Save map Successfully");
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to save map");
  // }

  /*****
   * use navigation2 nav2_map_server
   */
  //////Need to launch : ros2 launch nav2_map_server map_saver_server.launch.py
  std::shared_ptr<rclcpp::Node> map_saver_node = rclcpp::Node::make_shared("map_saver_client");
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr map_saver_client =
    map_saver_node->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");

  auto save_map_request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();

  while (!map_saver_client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the SaveMap service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SaveMap service not available, waiting again...");
  }

  save_map_request->map_topic = "/map";
  rclcpp::Time now = map_saver_node->get_clock()->now(); 
  int t_s = (int)now.seconds() + 8*3600; //UTC+8: 8*3600
  long sod = t_s%(3600*24);
  int hh = sod/3600;
  long moh = sod%3600;
  int mm = moh/60;
  int ss = moh%60;
  string hh_str = hh > 10 ? to_string(hh) : '0' + to_string(hh);
  string mm_str = mm > 10 ? to_string(mm) : '0' + to_string(mm);
  string ss_str = ss > 10 ? to_string(ss) : '0' + to_string(ss);
  string package_share_directory = ament_index_cpp::get_package_share_directory("map_server_extension");
  save_map_request->map_url = package_share_directory + "/maps/map_" + hh_str + mm_str + ss_str;

  
  auto map_saver_result = map_saver_client->async_send_request(save_map_request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(map_saver_node, map_saver_result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Save map Successfully");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to save map");
  }

  rclcpp::shutdown();
  return 0;
}