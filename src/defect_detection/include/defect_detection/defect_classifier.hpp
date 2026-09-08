#ifndef DEFECT_DETECTION_DEFECT_CLASSIFIER_HPP
#define DEFECT_DETECTION_DEFECT_CLASSIFIER_HPP

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/ml.hpp>
#include <vector>
#include <map>

namespace defect_detection {

enum class DefectType {
    NONE,
    NORMAL,
    DENT,
    BULGE,
    SCRATCH,
    CRACK,
    UNKNOWN
};

struct DefectInfo {
    DefectType type;
    float confidence;
    Eigen::Vector4f position;
    float severity;
    std::string description;
};

class DefectClassifier : public rclcpp::Node {
public:
    DefectClassifier(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    // Classify defect region
    DefectInfo classifyDefect(const pcl::PointCloud<pcl::PointXYZ>::Ptr defect_cloud);
    
    // Classify multiple defects
    std::vector<DefectInfo> classifyDefects(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& defect_clouds);
    
    // Extract FPFH features for classification
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr extractFPFH(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Train classifier with labeled data
    bool trainClassifier(const std::string& training_data_path);
    
    // Load pre-trained SVM model
    bool loadModel(const std::string& model_path);
    
    // Save trained model
    bool saveModel(const std::string& model_path);

private:
    // Calculate geometric features for SVM
    std::vector<float> extractFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Calculate geometric features
    float calculateCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    float calculateRoughness(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    float calculateVolume(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    float calculateAspectRatio(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Rule-based classification (fallback)
    DefectType ruleBasedClassification(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // SVM-based classification
    DefectType svmClassification(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Convert label to DefectType
    DefectType labelToDefectType(int label);
    
    // Feature extraction parameters
    double fpfh_radius_search_;
    double normal_radius_search_;
    
    // Classification thresholds
    double scratch_curvature_threshold_;
    double dent_depth_threshold_;
    double crack_length_threshold_;
    double deformation_deviation_threshold_;
    
    // SVM model
    cv::Ptr<cv::ml::SVM> svm_;
    bool use_svm_;
    std::string model_path_;
};

} // namespace defect_detection

#endif // DEFECT_DETECTION_DEFECT_CLASSIFIER_HPP
