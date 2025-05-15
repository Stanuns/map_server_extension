#include <fstream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/map_server.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

class GetMapFileClientTest : public rclcpp::Node {
public:
    GetMapFileClientTest() : Node("get_map_file_client_test") {
        client_ = this->create_client<robot_interfaces::srv::MapServer>("/service_map");
        output_dir = ament_index_cpp::get_package_share_directory("map_server_extension");
    }

    bool getAndSaveMap(const std::string& map_name) { //, const std::string& output_dir = "./"
        auto request = std::make_shared<robot_interfaces::srv::MapServer::Request>();
        request->map_name = map_name;
        request->cmd_name = 2;

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
        // Save PGM file
        bool r1 = save_pgm(map_name, response->map_data);
        // Save YAML file
        bool r2 = save_yaml(map_name, response->map_data);

        // try {
        //     // Save PGM file
        //     std::ofstream pgm_file(pgm_path, std::ios::binary);
        //     if (!pgm_file.is_open()) {
        //         return false;
        //     }

        //     // Write PGM header (P5 format)
        //     pgm_file << "P5\n";
        //     pgm_file << map_data.info.width << " " << map_data.info.height << "\n";
        //     pgm_file << "255\n"; // Max grayscale value

        //     // Write occupancy grid data (convert back to PGM values)
        //     for (const auto& cell : map_data.data) {
        //         uint8_t pixel = (cell == 100) ? 0 : (cell == 0) ? 254 : 205; // 205 for unknown (-1)
        //         pgm_file.write(reinterpret_cast<const char*>(&pixel), sizeof(pixel));
        //     }

        //     // Save YAML file
        //     YAML::Emitter yaml_emitter;
        //     yaml_emitter << YAML::BeginMap;
        //     yaml_emitter << YAML::Key << "image" << YAML::Value << map_name + ".pgm";
        //     yaml_emitter << YAML::Key << "mode" << YAML::Value << "trinary";
        //     yaml_emitter << YAML::Key << "resolution" << YAML::Value << map_data.info.resolution;
        //     yaml_emitter << YAML::Key << "origin" << YAML::Value 
        //                  << YAML::Flow << YAML::BeginSeq 
        //                  << map_data.info.origin.position.x 
        //                  << map_data.info.origin.position.y 
        //                  << map_data.info.origin.position.z 
        //                  << YAML::EndSeq;
        //     yaml_emitter << YAML::Key << "negate" << YAML::Value << 0;
        //     yaml_emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
        //     yaml_emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;
        //     yaml_emitter << YAML::EndMap;

        //     std::ofstream yaml_file(yaml_path);
        //     if (!yaml_file.is_open()) {
        //         RCLCPP_ERROR(this->get_logger(), "error");
        //         return false;
        //     }
        //     yaml_file << yaml_emitter.c_str();

        // } catch (const std::exception& e) {
        //     RCLCPP_ERROR(this->get_logger(), "error");
        //     return false;
        // }

        // if (!response->result) {
        //     RCLCPP_ERROR(this->get_logger(), "Service failed: %s", response->message.c_str());
        //     return false;
        // }
        // // Save YAML file
        // std::string yaml_path = output_dir + "/" + map_name + ".yaml";
        // if (!saveFile(yaml_path, response->yaml_content)) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to save YAML file: %s", yaml_path.c_str());
        //     return false;
        // }
        // // Save PGM file
        // std::string pgm_path = output_dir + "/" + map_name + ".pgm";
        // if (!saveFile(pgm_path, response->pgm_content)) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to save PGM file: %s", pgm_path.c_str());
        //     return false;
        // }

        if(!r1 || !r2){
            RCLCPP_ERROR(this->get_logger(), "Failed to save map file");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Map saved successfully: %s", 
                    map_name.c_str());
        return true;
    }

private:
    bool saveFile(const std::string& path, const std::vector<uint8_t>& content) {
        std::ofstream file(path, std::ios::binary);
        if (!file) return false;
        file.write(reinterpret_cast<const char*>(content.data()), content.size());
        return file.good();
    }

    rclcpp::Client<robot_interfaces::srv::MapServer>::SharedPtr client_;
    std::string output_dir;

    bool save_pgm(const std::string& map_name, const nav_msgs::msg::OccupancyGrid& map_data) {
        std::string pgm_path = output_dir + "/" + map_name + ".pgm";
        std::ofstream pgm_file(pgm_path, std::ios::binary);
        if (!pgm_file) {
            RCLCPP_ERROR(get_logger(), "Failed to create PGM file: %s", pgm_path.c_str());
            return false;
        }

        // Write PGM header (P5 format)
        pgm_file << "P5\n" << map_data.info.width << " " << map_data.info.height << "\n255\n";

        // Write pixel data (convert occupancy values to PGM)
        for (const auto& pixel : map_data.data) {
            uint8_t pgm_pixel;
            if (pixel == 100) pgm_pixel = 0;       // Occupied
            else if (pixel == 0) pgm_pixel = 254;  // Free
            else pgm_pixel = 205;                  // Unknown (-1)
            pgm_file.write(reinterpret_cast<const char*>(&pgm_pixel), sizeof(pgm_pixel));
        }
        return true;
    }

    bool save_yaml(const std::string& map_name, const nav_msgs::msg::OccupancyGrid& map_data) {
        std::string yaml_path = output_dir + "/" + map_name + ".yaml";
        YAML::Emitter yaml_emitter;

        yaml_emitter << YAML::BeginMap;
        yaml_emitter << YAML::Key << "image" << YAML::Value << map_name + ".pgm";
        yaml_emitter << YAML::Key << "mode" << YAML::Value << "trinary";
        yaml_emitter << YAML::Key << "resolution" << YAML::Value << map_data.info.resolution;
        yaml_emitter << YAML::Key << "origin" << YAML::Value 
                     << YAML::Flow << YAML::BeginSeq
                     << map_data.info.origin.position.x
                     << map_data.info.origin.position.y
                     << map_data.info.origin.position.z 
                     << YAML::EndSeq;
        yaml_emitter << YAML::Key << "negate" << YAML::Value << 0;
        yaml_emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
        yaml_emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;
        yaml_emitter << YAML::EndMap;

        std::ofstream yaml_file(yaml_path);
        if (!yaml_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "error");
            return false;
        }
        yaml_file << yaml_emitter.c_str();
        return true;
    }
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