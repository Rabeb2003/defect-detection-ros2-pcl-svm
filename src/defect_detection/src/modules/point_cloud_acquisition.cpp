#include "defect_detection/point_cloud_acquisition.hpp"

namespace defect_detection {

PointCloudAcquisition::PointCloudAcquisition(const rclcpp::NodeOptions& options)
    : Node("point_cloud_acquisition", options) {
    
    // Declare parameters
    this->declare_parameter("outlier_mean_k", 50.0);
    this->declare_parameter("outlier_std_thresh", 1.0);
    this->declare_parameter("voxel_leaf_size", 0.01);
    this->declare_parameter("enable_filtering", true);
    
    // Get parameters
    outlier_mean_k_ = this->get_parameter("outlier_mean_k").as_double();
    outlier_std_thresh_ = this->get_parameter("outlier_std_thresh").as_double();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    enable_filtering_ = this->get_parameter("enable_filtering").as_bool();
    
    // Create subscriber and publisher
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/input_cloud", 10,
        std::bind(&PointCloudAcquisition::cloudCallback, this, std::placeholders::_1));
    
    processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/processed_cloud", 10);
    
    RCLCPP_INFO(this->get_logger(), "PointCloudAcquisition node initialized");
}

void PointCloudAcquisition::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto processed_cloud = processPointCloud(msg);
    
    if (processed_cloud && !processed_cloud->empty()) {
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*processed_cloud, output_msg);
        output_msg.header = msg->header;
        processed_cloud_pub_->publish(output_msg);
        
        RCLCPP_INFO(this->get_logger(), "Processed cloud: %zu points", processed_cloud->size());
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudAcquisition::processPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (enable_filtering_) {
        cloud = removeOutliers(cloud);
        cloud = downsample(cloud);
    }
    
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudAcquisition::removeOutliers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    
    sor.setInputCloud(cloud);
    sor.setMeanK(outlier_mean_k_);
    sor.setStddevMulThresh(outlier_std_thresh_);
    sor.filter(*filtered);
    
    RCLCPP_DEBUG(this->get_logger(), "Outlier removal: %zu -> %zu points", 
                 cloud->size(), filtered->size());
    
    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudAcquisition::downsample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    
    vox.setInputCloud(cloud);
    vox.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    vox.filter(*filtered);
    
    RCLCPP_DEBUG(this->get_logger(), "Downsampling: %zu -> %zu points", 
                 cloud->size(), filtered->size());
    
    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudAcquisition::loadFromFile(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load file: %s", filename.c_str());
        return nullptr;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded cloud from %s: %zu points", 
                filename.c_str(), cloud->size());
    
    if (enable_filtering_) {
        cloud = removeOutliers(cloud);
        cloud = downsample(cloud);
    }
    
    return cloud;
}

bool PointCloudAcquisition::saveToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                       const std::string& filename) {
    if (pcl::io::savePCDFileBinary(filename, *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save file: %s", filename.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Saved cloud to %s: %zu points", 
                filename.c_str(), cloud->size());
    return true;
}

} // namespace defect_detection
