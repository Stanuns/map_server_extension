#include <fstream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/map_server.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

class UpdateMapFileClientTest : public rclcpp::Node {
public:
    UpdateMapFileClientTest() : Node("update_map_file_client_test") {
        client_ = this->create_client<robot_interfaces::srv::MapServer>("service_map");
        read_dir = ament_index_cpp::get_package_share_directory("map_server_extension");
    }

    bool readAndSendMap(const std::string& map_name) { //, const std::string& output_dir = "./"
        auto request = std::make_shared<robot_interfaces::srv::MapServer::Request>();
        request->map_name = map_name;
        request->cmd_name = 4;

        std::string yaml_path = read_dir  + "/" + map_name + ".yaml";
        std::string pgm_path = read_dir  + "/" + map_name + ".pgm";

        // if (!readFile(yaml_path, request->yaml_content)) {
        //     RCLCPP_INFO(this->get_logger(), "Failed to read YAML file: %s", yaml_path.c_str());
        //     return false;
        // }
        // if (!readFile(pgm_path, request->pgm_content)) {
        //     RCLCPP_INFO(this->get_logger(), "Failed to read PGM file: %s", pgm_path.c_str());
        //     return false;
        // }

        // Read YAML file
        YAML::Node yaml_node = YAML::LoadFile(yaml_path);
        request->map_data.info.resolution = yaml_node["resolution"].as<float>();
        request->map_data.info.origin.position.x = yaml_node["origin"][0].as<float>();
        request->map_data.info.origin.position.y = yaml_node["origin"][1].as<float>();
        request->map_data.info.origin.position.z = yaml_node["origin"][2].as<float>();
        request->map_data.info.origin.orientation.w = 1.0; // Default orientation
        // Read PGM file 
        std::ifstream pgm_file(pgm_path, std::ios::binary);
        if (!pgm_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open PGM file: %s", pgm_path.c_str());
            return false;
        }
        // Parse PGM header (P5 format)
        std::string magic_number;
        std::getline(pgm_file, magic_number); // Read "P5"
        if (magic_number != "P5") {
            RCLCPP_ERROR(this->get_logger(), "Invalid PGM format (not P5):  %s", pgm_path.c_str());
            return false;
        }
        // Skip comments and read width/height
        std::string line;
        while (std::getline(pgm_file, line)) {
            if (line.empty() || line[0] == '#') continue;
            break;
        }
        std::istringstream dimensions_stream(line);
        int width, height;
        dimensions_stream >> width >> height;
        // Read max_val and skip to binary data
        std::getline(pgm_file, line); // Read max_val line
        pgm_file.ignore(); // Skip newline after max_val

        request->map_data.info.width = width;
        request->map_data.info.height = height;
        // Read PGM data into occupancy grid
        request->map_data.data.resize(width * height);
        for (int i = 0; i < width * height; ++i) {
            uint8_t pixel;
            pgm_file.read(reinterpret_cast<char*>(&pixel), sizeof(pixel));
            request->map_data.data[i] = (pixel == 0) ? 100 : (pixel == 254) ? 0 : -1; // Convert to occupancy values
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
        if (response->err_code != 200) {
            RCLCPP_ERROR(get_logger(), "Error: %s", response->err_msg.c_str());
            return false;
        }
        // if (!response->result) {
        //     RCLCPP_ERROR(this->get_logger(), "Service failed: %s", response->err_msg.c_str());
        //     return false;
        // }

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

    rclcpp::Client<robot_interfaces::srv::MapServer>::SharedPtr client_;
    std::string read_dir;
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <map_name>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto client = std::make_shared<UpdateMapFileClientTest>();
    bool success = client->readAndSendMap(argv[1]);
    rclcpp::shutdown();
    return success ? 0 : 1;
}