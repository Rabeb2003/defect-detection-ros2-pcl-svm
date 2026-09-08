#include "defect_detection/defect_segmentation.hpp"

namespace defect_detection {

DefectSegmentation::DefectSegmentation(const rclcpp::NodeOptions& options)
    : Node("defect_segmentation", options) {
    
    // Declare parameters
    this->declare_parameter("plane_distance_threshold", 0.01);
    this->declare_parameter("plane_ransac_max_iterations", 100.0);
    this->declare_parameter("cluster_tolerance", 0.02);
    this->declare_parameter("min_cluster_size", 100.0);
    this->declare_parameter("max_cluster_size", 25000.0);
    this->declare_parameter("normal_search_radius", 0.05);
    this->declare_parameter("curvature_threshold", 0.1);
    
    // Get parameters
    plane_distance_threshold_ = this->get_parameter("plane_distance_threshold").as_double();
    plane_ransac_max_iterations_ = static_cast<int>(this->get_parameter("plane_ransac_max_iterations").as_double());
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = static_cast<int>(this->get_parameter("min_cluster_size").as_double());
    max_cluster_size_ = static_cast<int>(this->get_parameter("max_cluster_size").as_double());
    normal_search_radius_ = this->get_parameter("normal_search_radius").as_double();
    curvature_threshold_ = this->get_parameter("curvature_threshold").as_double();
    
    RCLCPP_INFO(this->get_logger(), "DefectSegmentation node initialized");
}

std::vector<DefectRegion> DefectSegmentation::segmentDefects(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    std::vector<DefectRegion> defects;
    
    // Segment planes first
    auto remaining_cloud = segmentPlanes(cloud);
    
    // Segment clusters from remaining points
    auto cluster_indices = segmentClusters(remaining_cloud);
    
    // Extract defect regions
    for (const auto& indices : cluster_indices) {
        DefectRegion region;
        region.indices = std::make_shared<pcl::PointIndices>(indices);
        region.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(region.indices);
        extract.setNegative(false);
        extract.filter(*region.cloud);
        
        // Calculate centroid
        pcl::compute3DCentroid(*region.cloud, region.centroid);
        
        // Calculate area (approximate)
        region.area = static_cast<float>(region.cloud->size()) * 0.001f; // rough estimate
        
        defects.push_back(region);
    }
    
    RCLCPP_INFO(this->get_logger(), "Segmented %zu defect regions", defects.size());
    return defects;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DefectSegmentation::segmentPlanes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(plane_ransac_max_iterations_);
    seg.setDistanceThreshold(plane_distance_threshold_);
    
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.empty()) {
        RCLCPP_WARN(this->get_logger(), "No plane found");
        return cloud;
    }
    
    // Extract remaining points (non-planar)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*remaining);
    
    RCLCPP_INFO(this->get_logger(), "Plane segmentation: %zu -> %zu points", 
                cloud->size(), remaining->size());
    
    return remaining;
}

std::vector<pcl::PointIndices> DefectSegmentation::segmentClusters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    std::vector<pcl::PointIndices> cluster_indices;
    
    // Create KD-tree for cluster extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    RCLCPP_INFO(this->get_logger(), "Found %zu clusters", cluster_indices.size());
    return cluster_indices;
}

pcl::PointCloud<pcl::Normal>::Ptr DefectSegmentation::estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    ne.setRadiusSearch(normal_search_radius_);
    ne.compute(*normals);
    
    return normals;
}

std::vector<pcl::PointIndices> DefectSegmentation::regionGrowingSegmentation(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr normals) {
    
    std::vector<pcl::PointIndices> cluster_indices;
    
    // Region growing not available in PCL 1.12, using Euclidean clustering instead
    RCLCPP_WARN(this->get_logger(), "Region growing not available, using Euclidean clustering");
    return segmentClusters(cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DefectSegmentation::calculateDistanceMap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cad_cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr distance_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Create KD-tree for CAD model
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cad_cloud);
    
    // For each point in input cloud, find nearest point in CAD
    for (const auto& point : input_cloud->points) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        
        tree->nearestKSearch(point, 1, indices, distances);
        
        pcl::PointXYZ p;
        p.x = point.x;
        p.y = point.y;
        p.z = distances[0]; // Store distance in z coordinate
        distance_cloud->push_back(p);
    }
    
    return distance_cloud;
}

} // namespace defect_detection
