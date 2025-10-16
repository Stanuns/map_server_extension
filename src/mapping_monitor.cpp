#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp> // 根据实际消息类型修改
#include <chrono>
#include "robot_interfaces/msg/mapping_state.hpp"

using namespace std::chrono_literals;

class MappingMonitor : public rclcpp::Node {
public:
  MappingMonitor() : Node("mapping_monitor") {
    last_map_time_ = this->now();
    has_map_data_ = false;

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/live_map", 10,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map_callback(msg);
      });
    
    state_pub_ = this->create_publisher<robot_interfaces::msg::MappingState>("/mapping_state", 10);
    
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MappingMonitor::check_map_status, this));
    
    RCLCPP_INFO(this->get_logger(), "mapping_monitor node have started");
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    last_map_time_ = this->now();
    has_map_data_ = true;
    
    // RCLCPP_DEBUG(this->get_logger(), "received map data");
  }

  void check_map_status() {
    auto time_since_last = this->now() - last_map_time_;
    auto time_since_last_ms = time_since_last.nanoseconds() / 1000000.0;
    
    if (time_since_last_ms > 2000.0) {
      has_map_data_ = false;
    }
    
    auto state_msg = robot_interfaces::msg::MappingState();
    state_msg.state = has_map_data_ ? 1 : 0;
    
    state_pub_->publish(state_msg);
    
    // //debug
    // if (has_map_data_ != last_state_) {
    //   if (has_map_data_) {
    //     RCLCPP_INFO(this->get_logger(), "检测到地图数据活动");
    //   } else {
    //     RCLCPP_WARN(this->get_logger(), "地图数据丢失 (%.1fms 无数据)", time_since_last_ms);
    //   }
    //   last_state_ = has_map_data_;
    // }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<robot_interfaces::msg::MappingState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Time last_map_time_;
  bool has_map_data_;
  bool last_state_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MappingMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}