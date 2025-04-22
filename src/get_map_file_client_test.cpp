#include <fstream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/get_map_file.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

class GetMapFileClientTest : public rclcpp::Node {
public:
    GetMapFileClientTest() : Node("get_map_file_client_test") {
        client_ = this->create_client<robot_interfaces::srv::GetMapFile>("get_map_file");
        output_dir = ament_index_cpp::get_package_share_directory("map_server_extension");
    }

    bool getAndSaveMap(const std::string& map_name) { //, const std::string& output_dir = "./"
        auto request = std::make_shared<robot_interfaces::srv::GetMapFile::Request>();
        request->map_name = map_name;

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto result = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
            return false;
        }

        auto response = result.get();

        if (!response->result) {
            RCLCPP_ERROR(this->get_logger(), "Service failed: %s", response->message.c_str());
            return false;
        }

        // Save YAML file
        std::string yaml_path = output_dir + "/" + map_name + ".yaml";
        if (!saveFile(yaml_path, response->yaml_content)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save YAML file: %s", yaml_path.c_str());
            return false;
        }

        // Save PGM file
        std::string pgm_path = output_dir + "/" + map_name + ".pgm";
        if (!saveFile(pgm_path, response->pgm_content)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save PGM file: %s", pgm_path.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully saved map files to: %s and %s", 
                    yaml_path.c_str(), pgm_path.c_str());
        return true;
    }

private:
    bool saveFile(const std::string& path, const std::vector<uint8_t>& content) {
        std::ofstream file(path, std::ios::binary);
        if (!file) return false;
        file.write(reinterpret_cast<const char*>(content.data()), content.size());
        return file.good();
    }

    rclcpp::Client<robot_interfaces::srv::GetMapFile>::SharedPtr client_;
    std::string output_dir;
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <map_name>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto client = std::make_shared<GetMapFileClientTest>();
    bool success = client->getAndSaveMap(argv[1]);
    rclcpp::shutdown();
    return success ? 0 : 1;
}