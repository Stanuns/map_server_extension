#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <dirent.h>
#include <vector>
#include <string>
#include <set>
#include <ament_index_cpp/get_package_share_directory.hpp>

/***
 * 1.获取当前maps的列表
 * 
 */
using namespace std;
class MapMgmtServer : public rclcpp::Node {
public:
    MapMgmtServer() : Node("map_mgmt_server") {
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "get_maps_name_list",
            std::bind(&MapMgmtServer::handle_request, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Service is ready to get maps name list.");
    }

private:
    void handle_request(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;  // Unused

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("map_server_extension");
        std::string maps_dir = package_share_directory + "/maps/";
        std::set<std::string> pgm_files;
        std::set<std::string> yaml_files;
        std::vector<std::string> valid_map_names;

        // Read directory
        DIR *dir = opendir(maps_dir.c_str());
        if (dir == nullptr) {
            response->success = false;
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
        response->success = true;
        response->message = "";
        for (const auto &name : valid_map_names) {
            response->message += name + ";";
        }
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapMgmtServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}