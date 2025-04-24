#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <dirent.h>
#include <vector>
#include <string>
#include <set>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_interfaces/srv/get_maps_name_list.hpp"
#include "robot_interfaces/srv/remove_map_file.hpp"
#include "robot_interfaces/srv/get_map_file.hpp"
#include "robot_interfaces/srv/update_map_file.hpp"

/***
 * 1.获取当前maps目录下各个map name列表
 * 2.传入地图名称，获取地图文件pgm、yaml文件
 * 3.下发(载入)地图文件
 * 4.删除地图
 */
using namespace std;
class MapMgmtServer : public rclcpp::Node {
public:
    MapMgmtServer() : Node("map_mgmt_server") {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("map_server_extension");
        maps_dir = package_share_directory + "/maps/";

        service_gmnl_ = this->create_service<robot_interfaces::srv::GetMapsNameList>(
            "get_maps_name_list",
            std::bind(&MapMgmtServer::handle_request_gmnl, this, std::placeholders::_1, std::placeholders::_2));

        service_rmf_ = this->create_service<robot_interfaces::srv::RemoveMapFile>(
            "remove_map_file",
            std::bind(&MapMgmtServer::handle_request_rmf, this, std::placeholders::_1, std::placeholders::_2));
        
        service_gmf_ = this->create_service<robot_interfaces::srv::GetMapFile>(
            "get_map_file",
            std::bind(&MapMgmtServer::handle_request_gmf, this, std::placeholders::_1, std::placeholders::_2));
        
        service_umf_ = this->create_service<robot_interfaces::srv::UpdateMapFile>(
            "update_map_file",
            std::bind(&MapMgmtServer::handle_request_umf, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Service is ready to map management.");
    }

private:
    void handle_request_gmnl(
        const std::shared_ptr<robot_interfaces::srv::GetMapsNameList::Request> request,
        std::shared_ptr<robot_interfaces::srv::GetMapsNameList::Response> response) {
        (void)request;  // Unused

        std::set<std::string> pgm_files;
        std::set<std::string> yaml_files;
        std::vector<std::string> valid_map_names;

        // Read directory
        DIR *dir = opendir(maps_dir.c_str());
        if (dir == nullptr) {
            response->result = false;
            response->message = "Failed to open maps directory.";
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
        response->result = true;
        response->name_list = "";
        for (const auto &name : valid_map_names) {
            response->name_list += name + ";";
        }
    }

    void handle_request_rmf(
        const std::shared_ptr<robot_interfaces::srv::RemoveMapFile::Request> request,
        std::shared_ptr<robot_interfaces::srv::RemoveMapFile::Response> response) {

        std::string map_name = request->map_name;
        
        // Construct file paths
        std::string pgm_path = maps_dir + map_name + ".pgm";
        std::string yaml_path = maps_dir + map_name + ".yaml";
        // Delete PGM file
        if (std::remove(pgm_path.c_str()) != 0) {
            response->result = false;
            response->message = "Failed to delete PGM file: " + map_name + ".pgm";
            return;
        }
        // Delete YAML file
        if (std::remove(yaml_path.c_str()) != 0) {
            response->result = false;
            response->message = "Failed to delete YAML file: " + map_name + ".yaml";
            return;
        }
        response->result = true;
        response->message = "Successfully deleted map files: " + map_name + ".pgm and " + map_name + ".yaml";
    }

    void handle_request_gmf(
        const std::shared_ptr<robot_interfaces::srv::GetMapFile::Request> request,
        std::shared_ptr<robot_interfaces::srv::GetMapFile::Response> response) {

        std::string map_name = request->map_name;
        // Read YAML file
        std::string yaml_path = maps_dir + map_name + ".yaml";
        if (!readFile(yaml_path, response->yaml_content)) {
            response->result = false;
            response->message = "Failed to read YAML file: " + yaml_path;
            return;
        }
        // Read PGM file
        std::string pgm_path = maps_dir + map_name + ".pgm";
        if (!readFile(pgm_path, response->pgm_content)) {
            response->result = false;
            response->message = "Failed to read PGM file: " + pgm_path;
            return;
        }
        response->result = true;
        response->message = "Successfully retrieved map files: " + map_name;          
    }

    void handle_request_umf(
        const std::shared_ptr<robot_interfaces::srv::UpdateMapFile::Request> request,
        std::shared_ptr<robot_interfaces::srv::UpdateMapFile::Response> response) {
                  
        std::string yaml_path = maps_dir + request->map_name + ".yaml";
        std::string pgm_path = maps_dir + request->map_name + ".pgm";
        try {
            // Save YAML file
            std::ofstream yaml_file(yaml_path, std::ios::binary);
            if (!yaml_file.is_open()) {
                throw std::runtime_error("Failed to open YAML file for writing");
            }
            yaml_file.write(reinterpret_cast<const char*>(request->yaml_content.data()), 
                          request->yaml_content.size());
            yaml_file.close();

            // Save PGM file
            std::ofstream pgm_file(pgm_path, std::ios::binary);
            if (!pgm_file.is_open()) {
                throw std::runtime_error("Failed to open PGM file for writing");
            }
            pgm_file.write(reinterpret_cast<const char*>(request->pgm_content.data()), 
                         request->pgm_content.size());
            pgm_file.close();

            // Set success response
            response->result = true;
            response->err_code = "";
            response->err_msg = "Files saved successfully";
        } catch (const std::exception& e) {
            // Set error response
            response->result = false;
            response->err_code = "FILE_SAVE_ERROR";
            response->err_msg = e.what();
            
            // Clean up partially written files if any
            filesystem::remove(yaml_path);
            filesystem::remove(pgm_path);
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

    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::Service<robot_interfaces::srv::GetMapsNameList>::SharedPtr service_gmnl_;
    rclcpp::Service<robot_interfaces::srv::RemoveMapFile>::SharedPtr service_rmf_;
    rclcpp::Service<robot_interfaces::srv::GetMapFile>::SharedPtr service_gmf_;
    rclcpp::Service<robot_interfaces::srv::UpdateMapFile>::SharedPtr service_umf_;
    std::string maps_dir;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapMgmtServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}