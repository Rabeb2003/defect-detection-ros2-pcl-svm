#ifndef DEFECT_DETECTION_CAD_REGISTRATION_HPP
#define DEFECT_DETECTION_CAD_REGISTRATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <Eigen/Dense>

namespace defect_detection {

class CADRegistration : public rclcpp::Node {
public:
    CADRegistration(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    // Load CAD model from file
    bool loadCADModel(const std::string& filename);
    
    // Register input cloud to CAD model using ICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr registerICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    
    // Register input cloud to CAD model using NDT
    pcl::PointCloud<pcl::PointXYZ>::Ptr registerNDT(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
    
    // Get transformation matrix
    Eigen::Matrix4f getTransformation() const { return final_transformation_; }
    
    // Get registration error
    float getRegistrationError() const { return registration_error_; }

private:
    // CAD model reference
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_model_;
    
    // Registration algorithms
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    
    // Results
    Eigen::Matrix4f final_transformation_;
    float registration_error_;
    
    // Parameters
    double icp_max_correspondence_distance_;
    int icp_max_iterations_;
    double icp_transformation_epsilon_;
    double icp_euclidean_fitness_epsilon_;
    
    double ndt_resolution_;
    int ndt_max_iterations_;
    double ndt_transformation_epsilon_;
    double ndt_step_size_;
    
    // Preprocess CAD model for faster registration
    void preprocessCADModel();
};

} // namespace defect_detection

#endif // DEFECT_DETECTION_CAD_REGISTRATION_HPP
