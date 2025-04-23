#include <fstream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/load_map_file.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

class LoadMapFileClientTest : public rclcpp::Node {
public:
    LoadMapFileClientTest() : Node("load_map_file_client_test") {
        client_ = this->create_client<robot_interfaces::srv::LoadMapFile>("load_map_file");
        read_dir = ament_index_cpp::get_package_share_directory("map_server_extension");
    }

    bool readAndSendMap(const std::string& map_name) { //, const std::string& output_dir = "./"
        auto request = std::make_shared<robot_interfaces::srv::LoadMapFile::Request>();
        request->map_name = map_name;

        std::string yaml_path = read_dir  + "/" + map_name + ".yaml";
        if (!readFile(yaml_path, request->yaml_content)) {
            RCLCPP_INFO(this->get_logger(), "Failed to read YAML file: %s", yaml_path.c_str());
            return false;
        }
        std::string pgm_path = read_dir  + "/" + map_name + ".pgm";
        if (!readFile(pgm_path, request->pgm_content)) {
            RCLCPP_INFO(this->get_logger(), "Failed to read PGM file: %s", pgm_path.c_str());
            return false;
        }
        

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
            RCLCPP_ERROR(this->get_logger(), "Service failed: %s", response->err_msg.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully send map files: %s and %s", 
                    yaml_path.c_str(), pgm_path.c_str());
        return true;
    }

private:
    bool readFile(const std::string& path, std::vector<uint8_t>& content) {
        std::ifstream file(path, std::ios::binary);
        if (!file) return false;

        file.seekg(0, std::ios::end);
        content.resize(file.tellg());
        file.seekg(0, std::ios::beg);
        file.read(reinterpret_cast<char*>(content.data()), content.size());
        return !file.fail();
    }

    rclcpp::Client<robot_interfaces::srv::LoadMapFile>::SharedPtr client_;
    std::string read_dir;
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <map_name>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto client = std::make_shared<LoadMapFileClientTest>();
    bool success = client->readAndSendMap(argv[1]);
    rclcpp::shutdown();
    return success ? 0 : 1;
}