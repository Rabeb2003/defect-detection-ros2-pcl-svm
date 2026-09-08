#include "defect_detection/cad_registration.hpp"
#include <pcl/io/pcd_io.h>

namespace defect_detection {

CADRegistration::CADRegistration(const rclcpp::NodeOptions& options)
    : Node("cad_registration", options) {
    
    // Declare parameters
    this->declare_parameter("icp_max_correspondence_distance", 0.05);
    this->declare_parameter("icp_max_iterations", 50.0);
    this->declare_parameter("icp_transformation_epsilon", 1e-8);
    this->declare_parameter("icp_euclidean_fitness_epsilon", 1e-6);
    this->declare_parameter("ndt_resolution", 0.1);
    this->declare_parameter("ndt_max_iterations", 35.0);
    this->declare_parameter("ndt_step_size", 0.1);
    
    // Get parameters
    icp_max_correspondence_distance_ = this->get_parameter("icp_max_correspondence_distance").as_double();
    icp_max_iterations_ = static_cast<int>(this->get_parameter("icp_max_iterations").as_double());
    icp_transformation_epsilon_ = this->get_parameter("icp_transformation_epsilon").as_double();
    icp_euclidean_fitness_epsilon_ = this->get_parameter("icp_euclidean_fitness_epsilon").as_double();
    ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
    ndt_max_iterations_ = static_cast<int>(this->get_parameter("ndt_max_iterations").as_double());
    ndt_step_size_ = this->get_parameter("ndt_step_size").as_double();
    
    // Initialize ICP
    icp_.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
    icp_.setMaximumIterations(icp_max_iterations_);
    icp_.setTransformationEpsilon(icp_transformation_epsilon_);
    icp_.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);
    
    // Initialize NDT
    ndt_.setResolution(ndt_resolution_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setMaximumIterations(ndt_max_iterations_);
    
    cad_model_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    final_transformation_ = Eigen::Matrix4f::Identity();
    registration_error_ = 0.0f;
    
    RCLCPP_INFO(this->get_logger(), "CADRegistration node initialized");
}

bool CADRegistration::loadCADModel(const std::string& filename) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cad_model_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load CAD model: %s", filename.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded CAD model: %s (%zu points)", 
                filename.c_str(), cad_model_->size());
    
    preprocessCADModel();
    return true;
}

void CADRegistration::preprocessCADModel() {
    // Downsample CAD model for faster registration
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_voxel;
    
    approx_voxel.setLeafSize(0.01, 0.01, 0.01);
    approx_voxel.setInputCloud(cad_model_);
    approx_voxel.filter(*filtered);
    
    cad_model_ = filtered;
    
    RCLCPP_INFO(this->get_logger(), "Preprocessed CAD model: %zu points", cad_model_->size());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CADRegistration::registerICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    
    if (cad_model_->empty()) {
        RCLCPP_ERROR(this->get_logger(), "CAD model not loaded");
        return nullptr;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    icp_.setInputSource(input_cloud);
    icp_.setInputTarget(cad_model_);
    icp_.align(*aligned_cloud);
    
    if (icp_.hasConverged()) {
        final_transformation_ = icp_.getFinalTransformation();
        registration_error_ = icp_.getFitnessScore();
        
        RCLCPP_INFO(this->get_logger(), "ICP converged with score: %f", registration_error_);
    } else {
        RCLCPP_WARN(this->get_logger(), "ICP did not converge");
    }
    
    return aligned_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CADRegistration::registerNDT(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
    
    if (cad_model_->empty()) {
        RCLCPP_ERROR(this->get_logger(), "CAD model not loaded");
        return nullptr;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    ndt_.setInputSource(input_cloud);
    ndt_.setInputTarget(cad_model_);
    ndt_.align(*aligned_cloud);
    
    if (ndt_.hasConverged()) {
        final_transformation_ = ndt_.getFinalTransformation();
        registration_error_ = ndt_.getFitnessScore();
        
        RCLCPP_INFO(this->get_logger(), "NDT converged with score: %f", registration_error_);
    } else {
        RCLCPP_WARN(this->get_logger(), "NDT did not converge");
    }
    
    return aligned_cloud;
}

} // namespace defect_detection
