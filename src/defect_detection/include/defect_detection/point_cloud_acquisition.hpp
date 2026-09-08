#ifndef DEFECT_DETECTION_POINT_CLOUD_ACQUISITION_HPP
#define DEFECT_DETECTION_POINT_CLOUD_ACQUISITION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace defect_detection {

class PointCloudAcquisition : public rclcpp::Node {
public:
    PointCloudAcquisition(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    // Process raw point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // Load point cloud from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadFromFile(const std::string& filename);
    
    // Save point cloud to file
    bool saveToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& filename);

private:
    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
    
    // Callback for incoming point clouds
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // Apply statistical outlier removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Apply voxel grid downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Parameters
    double outlier_mean_k_;
    double outlier_std_thresh_;
    double voxel_leaf_size_;
    bool enable_filtering_;
};

} // namespace defect_detection

#endif // DEFECT_DETECTION_POINT_CLOUD_ACQUISITION_HPP
