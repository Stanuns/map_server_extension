#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <dirent.h>
#include <vector>
#include <string>
#include <set>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robot_interfaces/srv/map_server.hpp"
#include "robot_interfaces/srv/save_map.hpp"
#include "nav2_msgs/srv/save_map.hpp"



/***
 *              1.获取当前maps目录下各个map name列表
 *              2.传入地图名称，获取地图文件pgm、yaml文件
 * /map_server  3.设置当前地图
 *              4.更新地图文件
 *              5.删除地图
 * /save_map    6.保存地图
 */
using namespace std;
class MapMgmtServer : public rclcpp::Node {
public:
    MapMgmtServer() : Node("map_mgmt_server") {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("map_server_extension");
        maps_dir = package_share_directory + "/maps/";
        params_dir = package_share_directory + "/params/";


        service_ms_ = this->create_service<robot_interfaces::srv::MapServer>(
            "/map_server",
            std::bind(&MapMgmtServer::handle_request_ms, this, std::placeholders::_1, std::placeholders::_2));
        
        service_sm_ = this->create_service<robot_interfaces::srv::SaveMap>(
            "/save_map",
            std::bind(&MapMgmtServer::handle_request_sm, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Service is ready to map management.");
    }

private:
    void handle_request_ms(
        const std::shared_ptr<robot_interfaces::srv::MapServer::Request> request,
        std::shared_ptr<robot_interfaces::srv::MapServer::Response> response) {
            uint32_t cmd = request->cmd_name;

            if (cmd == 1){ //get maps name list
                handle_request_gmnl(request, response);
            }else if(cmd == 2){//get map file
                handle_request_gmf(request, response);
            }else if(cmd == 3){//set current map
                handle_request_scm(request, response);
            }else if(cmd == 4){//update map file
                handle_request_umf(request, response);
            }else if(cmd == 5){//remove map file
                handle_request_rmf(request, response);
            }else{

            }
            
            
        }

    void handle_request_gmnl( //get maps name list
        const std::shared_ptr<robot_interfaces::srv::MapServer::Request> request,
        std::shared_ptr<robot_interfaces::srv::MapServer::Response> response) {

        std::set<std::string> pgm_files;
        std::set<std::string> yaml_files;
        std::vector<std::string> valid_map_names;

        // Read directory
        DIR *dir = opendir(maps_dir.c_str());
        if (dir == nullptr) {
            response->err_code = 500;
            response->err_msg = "Failed to open maps directory.";
            RCLCPP_ERROR(this->get_logger(), "Directory not found: %s", maps_dir.c_str());
            return;
        }

        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string filename = entry->d_name;
            if (filename == "." || filename == "..") continue;

            // Check for .pgm or .yaml extensions
            size_t dot_pos = filename.find_last_of(".");
            if (dot_pos != std::string::npos) {
                std::string basename = filename.substr(0, dot_pos);
                std::string extension = filename.substr(dot_pos + 1);

                if (extension == "pgm") {
                    pgm_files.insert(basename);
                } else if (extension == "yaml") {
                    yaml_files.insert(basename);
                }
            }
        }
        closedir(dir);

        // Find intersection (names that have both .pgm and .yaml)
        for (const auto &name : pgm_files) {
            if (yaml_files.find(name) != yaml_files.end()) {
                valid_map_names.push_back(name);
            }
        }

        // Prepare response
        response->err_code = 200;
        response->err_msg = "Successfully get maps name list";
        // for (const auto &name : valid_map_names) {
        //     response->map_list.push_back(name);
        // }
        response->map_list = valid_map_names;
    }

    void handle_request_rmf(
        const std::shared_ptr<robot_interfaces::srv::MapServer::Request> request,
        std::shared_ptr<robot_interfaces::srv::MapServer::Response> response) {

        std::string map_name = request->map_name;
        
        // Construct file paths
        std::string pgm_path = maps_dir + map_name + ".pgm";
        std::string yaml_path = maps_dir + map_name + ".yaml";
        // Delete PGM file
        if (std::remove(pgm_path.c_str()) != 0) {
            response->err_code = 500;
            response->err_msg = "Failed to delete PGM file: " + map_name + ".pgm";
            return;
        }
        // Delete YAML file
        if (std::remove(yaml_path.c_str()) != 0) {
            response->err_code = 500;
            response->err_msg = "Failed to delete YAML file: " + map_name + ".yaml";
            return;
        }
        response->err_code = 200;
        response->err_msg = "Successfully deleted map files: " + map_name + ".pgm and " + map_name + ".yaml";
    }

    void handle_request_gmf(
        const std::shared_ptr<robot_interfaces::srv::MapServer::Request> request,
        std::shared_ptr<robot_interfaces::srv::MapServer::Response> response) {

        std::string map_name = request->map_name;
        std::string yaml_path = maps_dir + map_name + ".yaml";
        std::string pgm_path = maps_dir + map_name + ".pgm";

        try {
            // Read YAML file
            YAML::Node yaml_node = YAML::LoadFile(yaml_path);
            response->map_data.info.resolution = yaml_node["resolution"].as<float>();
            response->map_data.info.origin.position.x = yaml_node["origin"][0].as<float>();
            response->map_data.info.origin.position.y = yaml_node["origin"][1].as<float>();
            response->map_data.info.origin.position.z = yaml_node["origin"][2].as<float>();
            response->map_data.info.origin.orientation.w = 1.0; // Default orientation

            // Read PGM file 
            std::ifstream pgm_file(pgm_path, std::ios::binary);
            if (!pgm_file.is_open()) {
                response->err_code = 500;
                response->err_msg = "Failed to open PGM file: " + pgm_path;
                return;
            }
            // Parse PGM header (P5 format)
            std::string magic_number;
            std::getline(pgm_file, magic_number); // Read "P5"
            if (magic_number != "P5") {
                response->err_code = 500;
                response->err_msg = "Invalid PGM format (not P5): " + pgm_path;
                return;
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

            response->map_data.info.width = width;
            response->map_data.info.height = height;
            // Read PGM data into occupancy grid
            response->map_data.data.resize(width * height);
            for (int i = 0; i < width * height; ++i) {
                uint8_t pixel;
                pgm_file.read(reinterpret_cast<char*>(&pixel), sizeof(pixel));
                response->map_data.data[i] = (pixel == 0) ? 100 : (pixel == 254) ? 0 : -1; // Convert to occupancy values
            }

            response->err_code = 200;
            response->err_msg = "Successfully retrieved map: " + map_name;

        } catch (const YAML::Exception& e) {
            response->err_code = 500;
            response->err_msg = "YAML parsing error: " + std::string(e.what());
        } catch (const std::exception& e) {
            response->err_code = 500;
            response->err_msg = "Error reading map files: " + std::string(e.what());
        }
           
    }

    void handle_request_umf(
        const std::shared_ptr<robot_interfaces::srv::MapServer::Request> request,
        std::shared_ptr<robot_interfaces::srv::MapServer::Response> response) {
                  
        string map_name = request->map_name;
        
        try {
            // Save PGM file
            save_pgm(map_name, request->map_data);
            // Save YAML file
            save_yaml(map_name, request->map_data);

            // Set success response
            response->err_code = 200;
            response->err_msg = "Map file updated successfully";
        } catch (const std::exception& e) {
            // Set error response
            response->err_code = 500;
            response->err_msg = "Failed to update map file: " + std::string(e.what());
        
        }
    }

    void handle_request_scm( //set current map
        const std::shared_ptr<robot_interfaces::srv::MapServer::Request> request,
        std::shared_ptr<robot_interfaces::srv::MapServer::Response> response) {
                  
        try {
            // Load the existing YAML file
            YAML::Node config = YAML::LoadFile(params_dir + "map_mgmt.yaml");
            
            // Update the current_map_name value
            config["map_mgmt_server"]["ros__parameters"]["current_map_name"] = request->map_name;
            
            // Write back to the file
            std::ofstream fout(params_dir + "map_mgmt.yaml");
            fout << config;
            fout.close();
            
            response->err_code = 200;
            response->err_msg = "Successfully set current_map_name to " + request->map_name;
        } catch (const std::exception& e) {
            response->err_code = 500;
            response->err_msg = "Failed to set map_mgmt.yaml: " + std::string(e.what());
        }
    
    }

    void handle_request_sm(
        const std::shared_ptr<robot_interfaces::srv::SaveMap::Request> request,
        std::shared_ptr<robot_interfaces::srv::SaveMap::Response> response) {
            

            string map_name = request->map_name;

            //Need to launch : ros2 launch nav2_map_server map_saver_server.launch.py
            //creat a service client to call navigation2 nav2_map_server
            auto client_node = std::make_shared<rclcpp::Node>("temp_map_saver_client");
            rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr map_saver_client =
                client_node->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
            auto save_map_request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
            while (!map_saver_client->wait_for_service(2s)) {
                if (!rclcpp::ok()) {
                  RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the SaveMap service. Exiting.");
                  return;
                }
                RCLCPP_INFO(get_logger(), "SaveMap service not available, waiting again...");
            }
            save_map_request->map_topic = "/map";
            save_map_request->map_url = maps_dir + map_name;
            auto map_saver_result = map_saver_client->async_send_request(save_map_request);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(client_node, map_saver_result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                response->result = true;
                response->message = "Successfully saved map, map name: " + map_name;
                RCLCPP_INFO(get_logger(), "Save map Successfully");
            } else {
                response->result = false;
                response->message = "Failed to save map";
                RCLCPP_ERROR(get_logger(), "Failed to save map");
            }

    }

    bool readFile(const std::string& path, std::vector<uint8_t>& content) {
        std::ifstream file(path, std::ios::binary);
        if (!file) return false;

        file.seekg(0, std::ios::end);
        content.resize(file.tellg());
        file.seekg(0, std::ios::beg);
        file.read(reinterpret_cast<char*>(content.data()), content.size());
        return !file.fail();
    }

    rclcpp::Service<robot_interfaces::srv::MapServer>::SharedPtr service_ms_;
    rclcpp::Service<robot_interfaces::srv::SaveMap>::SharedPtr service_sm_;
    std::string maps_dir;
    std::string params_dir;

    void save_pgm(const std::string& map_name, const nav_msgs::msg::OccupancyGrid& map_data) {
        std::string pgm_path = maps_dir + map_name + ".pgm";
        std::ofstream pgm_file(pgm_path, std::ios::binary);
        if (!pgm_file) {
            std::string error_message = "Failed to create PGM file: " + pgm_path;
            RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
            throw std::runtime_error(error_message);
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
        
        if (!pgm_file.good()) {
            std::string error_message = "Failed to write PGM data to file: " + pgm_path;
            RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
            throw std::runtime_error(error_message);
        }
        
    }

    void save_yaml(const std::string& map_name, const nav_msgs::msg::OccupancyGrid& map_data) {
        std::string yaml_path = maps_dir + map_name + ".yaml";
        YAML::Emitter yaml_emitter;

        try {
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
                throw std::runtime_error("Failed to open YAML file for writing: " + yaml_path);
            }
            yaml_file << yaml_emitter.c_str();
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("YAML processing error: " + std::string(e.what()));
        } catch (const std::exception& e) {
            throw std::runtime_error("Error saving YAML file: " + std::string(e.what()));
        }
        
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapMgmtServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}