#include <rclcpp/rclcpp.hpp>
#include "defect_detection/point_cloud_acquisition.hpp"
#include "defect_detection/cad_registration.hpp"
#include "defect_detection/defect_segmentation.hpp"
#include "defect_detection/defect_classifier.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace defect_detection;

class DefectDetectionNode : public rclcpp::Node {
public:
    DefectDetectionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("defect_detection_node", options) {
        
        // Declare parameters
        this->declare_parameter("cad_model_path", "");
        this->declare_parameter("use_icp", true);
        this->declare_parameter("enable_classification", true);
        this->declare_parameter("publish_markers", true);
        
        // Get parameters
        cad_model_path_ = this->get_parameter("cad_model_path").as_string();
        use_icp_ = this->get_parameter("use_icp").as_bool();
        enable_classification_ = this->get_parameter("enable_classification").as_bool();
        publish_markers_ = this->get_parameter("publish_markers").as_bool();
        
        // Initialize modules
        acquisition_ = std::make_shared<PointCloudAcquisition>();
        registration_ = std::make_shared<CADRegistration>();
        segmentation_ = std::make_shared<DefectSegmentation>();
        classifier_ = std::make_shared<DefectClassifier>();
        
        // Load CAD model if path provided
        if (!cad_model_path_.empty()) {
            if (registration_->loadCADModel(cad_model_path_)) {
                RCLCPP_INFO(this->get_logger(), "CAD model loaded successfully");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to load CAD model, running without registration");
            }
        }
        
        // Create ROS2 interfaces
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/input_cloud", 10,
            std::bind(&DefectDetectionNode::processCloud, this, std::placeholders::_1));
        
        defect_pub_ = this->create_publisher<std_msgs::msg::String>("/defects_detected", 10);
        processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_cloud", 10);
        
        if (publish_markers_) {
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/defect_markers", 10);
        }
        
        RCLCPP_INFO(this->get_logger(), "DefectDetectionNode initialized");
    }

private:
    void processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Processing point cloud...");
        
        // Step 1: Acquire and preprocess
        auto processed_cloud = acquisition_->processPointCloud(msg);
        if (!processed_cloud || processed_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty cloud after preprocessing");
            return;
        }
        
        // Step 2: Register to CAD model
        pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud;
        if (!cad_model_path_.empty()) {
            if (use_icp_) {
                registered_cloud = registration_->registerICP(processed_cloud);
            } else {
                registered_cloud = registration_->registerNDT(processed_cloud);
            }
            
            if (registered_cloud && registration_->getRegistrationError() < 0.1) {
                processed_cloud = registered_cloud;
                RCLCPP_INFO(this->get_logger(), "Registration successful, error: %f", 
                           registration_->getRegistrationError());
            } else {
                RCLCPP_WARN(this->get_logger(), "Registration failed, using original cloud");
            }
        }
        
        // Step 3: Segment defects
        auto defect_regions = segmentation_->segmentDefects(processed_cloud);
        
        if (defect_regions.empty()) {
            RCLCPP_INFO(this->get_logger(), "No defects detected");
            publishProcessedCloud(processed_cloud);
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Detected %zu potential defects", defect_regions.size());
        
        // Step 4: Classify defects
        if (enable_classification_) {
            std::vector<DefectInfo> defect_infos;
            for (const auto& region : defect_regions) {
                defect_infos.push_back(classifier_->classifyDefect(region.cloud));
            }
            
            // Publish results
            publishDefectResults(defect_infos);
            
            if (publish_markers_) {
                publishDefectMarkers(defect_infos);
            }
        } else {
            // Publish raw defect regions
            std::string result_msg = "Detected " + std::to_string(defect_regions.size()) + " defect regions";
            auto msg_result = std_msgs::msg::String();
            msg_result.data = result_msg;
            defect_pub_->publish(msg_result);
        }
        
        // Publish processed cloud
        publishProcessedCloud(processed_cloud);
    }
    
    void publishDefectResults(const std::vector<DefectInfo>& defect_infos) {
        std::string result = "Defect Detection Results:\n";
        
        for (size_t i = 0; i < defect_infos.size(); ++i) {
            const auto& info = defect_infos[i];
            result += "Defect " + std::to_string(i + 1) + ": " + info.description + "\n";
            result += "  Position: [" + std::to_string(info.position[0]) + ", " + 
                      std::to_string(info.position[1]) + ", " + std::to_string(info.position[2]) + "]\n";
            result += "  Confidence: " + std::to_string(info.confidence) + "\n";
            result += "  Severity: " + std::to_string(info.severity) + "\n\n";
        }
        
        auto msg = std_msgs::msg::String();
        msg.data = result;
        defect_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Published defect detection results");
    }
    
    void publishDefectMarkers(const std::vector<DefectInfo>& defect_infos) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < defect_infos.size(); ++i) {
            const auto& info = defect_infos[i];
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "defects";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = info.position[0];
            marker.pose.position.y = info.position[1];
            marker.pose.position.z = info.position[2];
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            
            // Color based on severity
            marker.color.r = info.severity;
            marker.color.g = 1.0 - info.severity;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            
            marker_array.markers.push_back(marker);
        }
        
        marker_pub_->publish(marker_array);
    }
    
    void publishProcessedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = "base_link";
        msg.header.stamp = this->now();
        processed_cloud_pub_->publish(msg);
    }
    
    // Modules
    std::shared_ptr<PointCloudAcquisition> acquisition_;
    std::shared_ptr<CADRegistration> registration_;
    std::shared_ptr<DefectSegmentation> segmentation_;
    std::shared_ptr<DefectClassifier> classifier_;
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr defect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Parameters
    std::string cad_model_path_;
    bool use_icp_;
    bool enable_classification_;
    bool publish_markers_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DefectDetectionNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Defect Detection Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
