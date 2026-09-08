#include "defect_detection/defect_classifier.hpp"
#include <cmath>

namespace defect_detection {

DefectClassifier::DefectClassifier(const rclcpp::NodeOptions& options)
    : Node("defect_classifier", options) {
    
    // Declare parameters
    this->declare_parameter("fpfh_radius_search", 0.05);
    this->declare_parameter("normal_radius_search", 0.05);
    this->declare_parameter("scratch_curvature_threshold", 0.5);
    this->declare_parameter("dent_depth_threshold", 0.02);
    this->declare_parameter("crack_length_threshold", 0.1);
    this->declare_parameter("deformation_deviation_threshold", 0.05);
    this->declare_parameter("svm_model_path", "/home/rabeb/defect_detection_ws/data/svm_model.xml");
    this->declare_parameter("use_svm", true);
    
    // Get parameters
    fpfh_radius_search_ = this->get_parameter("fpfh_radius_search").as_double();
    normal_radius_search_ = this->get_parameter("normal_radius_search").as_double();
    scratch_curvature_threshold_ = this->get_parameter("scratch_curvature_threshold").as_double();
    dent_depth_threshold_ = this->get_parameter("dent_depth_threshold").as_double();
    crack_length_threshold_ = this->get_parameter("crack_length_threshold").as_double();
    deformation_deviation_threshold_ = this->get_parameter("deformation_deviation_threshold").as_double();
    model_path_ = this->get_parameter("svm_model_path").as_string();
    use_svm_ = this->get_parameter("use_svm").as_bool();
    
    // Initialize SVM
    svm_ = cv::ml::SVM::create();
    if (use_svm_) {
        if (loadModel(model_path_)) {
            RCLCPP_INFO(this->get_logger(), "SVM model loaded successfully");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load SVM model, using rule-based classification");
            use_svm_ = false;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "DefectClassifier node initialized");
}

DefectInfo DefectClassifier::classifyDefect(const pcl::PointCloud<pcl::PointXYZ>::Ptr defect_cloud) {
    DefectInfo info;
    info.type = DefectType::NONE;
    info.confidence = 0.0f;
    info.severity = 0.0f;
    
    if (!defect_cloud || defect_cloud->empty()) {
        return info;
    }
    
    // Calculate centroid
    pcl::compute3DCentroid(*defect_cloud, info.position);
    
    // Use SVM if available, otherwise use rule-based
    if (use_svm_) {
        info.type = svmClassification(defect_cloud);
        info.confidence = 0.78f; // Training accuracy
    } else {
        info.type = ruleBasedClassification(defect_cloud);
        // Set confidence based on feature strength
        float curvature = calculateCurvature(defect_cloud);
        float roughness = calculateRoughness(defect_cloud);
        info.confidence = std::min(1.0f, curvature + roughness);
    }
    
    // Calculate severity
    float volume = calculateVolume(defect_cloud);
    float aspect_ratio = calculateAspectRatio(defect_cloud);
    info.severity = std::min(1.0f, volume * aspect_ratio);
    
    // Set description
    switch (info.type) {
        case DefectType::NORMAL:
            info.description = "No defect detected";
            break;
        case DefectType::SCRATCH:
            info.description = "Surface scratch detected";
            break;
        case DefectType::DENT:
            info.description = "Dent deformation detected";
            break;
        case DefectType::BULGE:
            info.description = "Bulge deformation detected";
            break;
        case DefectType::CRACK:
            info.description = "Crack detected";
            break;
        default:
            info.description = "Unknown defect";
            break;
    }
    
    return info;
}

std::vector<DefectInfo> DefectClassifier::classifyDefects(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& defect_clouds) {
    
    std::vector<DefectInfo> results;
    
    for (const auto& cloud : defect_clouds) {
        results.push_back(classifyDefect(cloud));
    }
    
    return results;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr DefectClassifier::extractFPFH(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
    
    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(normal_radius_search_);
    ne.compute(*normals);
    
    // Extract FPFH features
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setRadiusSearch(fpfh_radius_search_);
    fpfh.compute(*features);
    
    return features;
}

DefectType DefectClassifier::ruleBasedClassification(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float curvature = calculateCurvature(cloud);
    float roughness = calculateRoughness(cloud);
    float volume = calculateVolume(cloud);
    float aspect_ratio = calculateAspectRatio(cloud);
    
    // Simple rule-based classification
    if (curvature > scratch_curvature_threshold_ && roughness < 0.3) {
        return DefectType::SCRATCH;
    } else if (volume < dent_depth_threshold_ && aspect_ratio > 2.0) {
        return DefectType::DENT;
    } else if (aspect_ratio > 5.0 && roughness > 0.5) {
        return DefectType::CRACK;
    } else if (roughness > deformation_deviation_threshold_) {
        return DefectType::BULGE;
    }
    
    return DefectType::UNKNOWN;
}

std::vector<float> DefectClassifier::extractFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::vector<float> features;
    
    // Feature 1: Number of points
    features.push_back(static_cast<float>(cloud->size()));
    
    // Feature 2-4: Bounding box dimensions
    float min_x = cloud->points[0].x, max_x = cloud->points[0].x;
    float min_y = cloud->points[0].y, max_y = cloud->points[0].y;
    float min_z = cloud->points[0].z, max_z = cloud->points[0].z;
    
    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    
    features.push_back(max_x - min_x);
    features.push_back(max_y - min_y);
    features.push_back(max_z - min_z);
    
    // Feature 5: Volume
    float volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
    features.push_back(volume);
    
    // Feature 6: Centroid z-coordinate
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    features.push_back(centroid[2]);
    
    // Feature 7: Normal statistics (curvature)
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(normal_radius_search_);
    ne.compute(*normals);
    
    float avg_curvature = 0.0f;
    for (const auto& normal : normals->points) {
        avg_curvature += normal.curvature;
    }
    avg_curvature /= normals->size();
    features.push_back(avg_curvature);
    
    // Feature 8: Standard deviation of z
    float mean_z = centroid[2];
    float var_z = 0.0f;
    for (const auto& point : cloud->points) {
        var_z += (point.z - mean_z) * (point.z - mean_z);
    }
    var_z /= cloud->size();
    features.push_back(std::sqrt(var_z));
    
    return features;
}

DefectType DefectClassifier::svmClassification(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::vector<float> features = extractFeatures(cloud);
    
    cv::Mat sample(1, features.size(), CV_32F);
    for (size_t i = 0; i < features.size(); i++) {
        sample.at<float>(0, i) = features[i];
    }
    
    int label = svm_->predict(sample);
    return labelToDefectType(label);
}

DefectType DefectClassifier::labelToDefectType(int label) {
    switch (label) {
        case 0: return DefectType::NORMAL;
        case 1: return DefectType::DENT;
        case 2: return DefectType::BULGE;
        case 3: return DefectType::SCRATCH;
        case 4: return DefectType::CRACK;
        default: return DefectType::UNKNOWN;
    }
}

float DefectClassifier::calculateCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->size() < 3) return 0.0f;
    
    // Estimate normals and curvature
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(normal_radius_search_);
    ne.compute(*normals);
    
    // Average curvature
    float total_curvature = 0.0f;
    int count = 0;
    
    for (const auto& normal : normals->points) {
        total_curvature += normal.curvature;
        count++;
    }
    
    return count > 0 ? total_curvature / count : 0.0f;
}

float DefectClassifier::calculateRoughness(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->size() < 2) return 0.0f;
    
    // Calculate centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    
    // Calculate variance from centroid
    float total_distance = 0.0f;
    for (const auto& point : cloud->points) {
        float dx = point.x - centroid[0];
        float dy = point.y - centroid[1];
        float dz = point.z - centroid[2];
        total_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    return total_distance / cloud->size();
}

float DefectClassifier::calculateVolume(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->size() < 4) return 0.0f;
    
    // Bounding box volume approximation
    float min_x = cloud->points[0].x, max_x = cloud->points[0].x;
    float min_y = cloud->points[0].y, max_y = cloud->points[0].y;
    float min_z = cloud->points[0].z, max_z = cloud->points[0].z;
    
    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float dz = max_z - min_z;
    
    return dx * dy * dz;
}

float DefectClassifier::calculateAspectRatio(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->size() < 2) return 1.0f;
    
    // Calculate bounding box dimensions
    float min_x = cloud->points[0].x, max_x = cloud->points[0].x;
    float min_y = cloud->points[0].y, max_y = cloud->points[0].y;
    float min_z = cloud->points[0].z, max_z = cloud->points[0].z;
    
    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float dz = max_z - min_z;
    
    float max_dim = std::max({dx, dy, dz});
    float min_dim = std::min({dx, dy, dz});
    
    return min_dim > 0.001f ? max_dim / min_dim : 1.0f;
}

bool DefectClassifier::trainClassifier(const std::string& training_data_path) {
    RCLCPP_INFO(this->get_logger(), "Training classifier with data from: %s", 
                training_data_path.c_str());
    // Training is done by separate executable
    return true;
}

bool DefectClassifier::loadModel(const std::string& model_path) {
    RCLCPP_INFO(this->get_logger(), "Loading model from: %s", model_path.c_str());
    
    try {
        svm_ = cv::ml::SVM::load(model_path);
        if (svm_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load SVM model");
            return false;
        }
        return true;
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        return false;
    }
}

bool DefectClassifier::saveModel(const std::string& model_path) {
    RCLCPP_INFO(this->get_logger(), "Saving model to: %s", model_path.c_str());
    try {
        svm_->save(model_path);
        return true;
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        return false;
    }
}

} // namespace defect_detection
