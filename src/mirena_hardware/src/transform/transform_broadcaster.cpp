#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class TransformBroadcasterNode : public rclcpp::Node {
public:
    TransformBroadcasterNode() : Node("mirena_transform") {
        // Empty means "use the copy installed with this package", resolved via the
        // ament index so it works from any install/overlay prefix.
        this->declare_parameter("yaml_path", "");
        // use_sim_time is a read-only parameter set by the launch system.
        // It must NOT be declared here: rclcpp already registers it, and a
        // second declare_parameter call throws. The node picks it up
        // automatically from the launch override (use_sim_time:=true).

        yaml_path_ = this->get_parameter("yaml_path").as_string();
        if (yaml_path_.empty()) {
            yaml_path_ = default_yaml_path();
        }

        if (yaml_path_.empty() || !parse_yaml(yaml_path_)) {
            RCLCPP_ERROR(get_logger(), "Configuration Failed: Could not parse YAML file at %s", yaml_path_.c_str());
            exit(39);
        }

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(), "Parsed %zu static transforms. Configuration Successful.", transforms_.size());
            // Update timestamps to current time before publishing
            auto now = this->get_clock()->now();
            for (auto & tf : transforms_) {
                tf.header.stamp = now;
            }

            // Publish all transforms to /tf_static
            tf_broadcaster_->sendTransform(transforms_);
            RCLCPP_INFO(get_logger(), "Published %zu static transforms.", transforms_.size());
        }

private: 
    std::string yaml_path_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    std::vector<geometry_msgs::msg::TransformStamped> transforms_;

    std::string default_yaml_path() {
        try {
            return ament_index_cpp::get_package_share_directory("mirena_hardware")
                   + "/config/transforms.yaml";
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Could not locate the mirena_hardware share directory: %s", e.what());
            return "";
        }
    }

    bool parse_yaml(const std::string& path) {
        try {
            YAML::Node config = YAML::LoadFile(path);
            
            if (!config["transforms"]) {
                RCLCPP_ERROR(get_logger(), "YAML file missing 'transforms' root key.");
                return false;
            }

            for (const auto& node : config["transforms"]) {
                geometry_msgs::msg::TransformStamped tf;
                
                tf.header.frame_id = node["parent_frame_id"].as<std::string>();
                tf.child_frame_id = node["child_frame_id"].as<std::string>();

                tf.transform.translation.x = node["translation"][0].as<double>();
                tf.transform.translation.y = node["translation"][1].as<double>();
                tf.transform.translation.z = node["translation"][2].as<double>();

                tf.transform.rotation.x = node["rotation"][0].as<double>();
                tf.transform.rotation.y = node["rotation"][1].as<double>();
                tf.transform.rotation.z = node["rotation"][2].as<double>();
                tf.transform.rotation.w = node["rotation"][3].as<double>();

                transforms_.push_back(tf);
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(get_logger(), "YAML Exception: %s", e.what());
            return false;
        }
        return true;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformBroadcasterNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
