#ifndef DEFECT_DETECTION_DEFECT_SEGMENTATION_HPP
#define DEFECT_DETECTION_DEFECT_SEGMENTATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

namespace defect_detection {

struct DefectRegion {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointIndices::Ptr indices;
    Eigen::Vector4f centroid;
    float area;
    std::string type;
};

class DefectSegmentation : public rclcpp::Node {
public:
    DefectSegmentation(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    // Segment defects from registered point cloud
    std::vector<DefectRegion> segmentDefects(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Segment planar surfaces
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Segment clusters (potential defects)
    std::vector<pcl::PointIndices> segmentClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Calculate distance map from CAD model
    pcl::PointCloud<pcl::PointXYZ>::Ptr calculateDistanceMap(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud);

private:
    // Estimate normals for curvature-based segmentation
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Region growing segmentation
    std::vector<pcl::PointIndices> regionGrowingSegmentation(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr normals);
    
    // Parameters
    double plane_distance_threshold_;
    int plane_ransac_max_iterations_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double normal_search_radius_;
    double curvature_threshold_;
};

} // namespace defect_detection

#endif // DEFECT_DETECTION_DEFECT_SEGMENTATION_HPP
