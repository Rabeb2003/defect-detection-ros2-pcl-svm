#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <opencv2/ml.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

struct TrainingSample {
    std::string filename;
    int label;
    std::vector<float> features;
};

class SVMTrainer {
public:
    SVMTrainer() {
        svm = cv::ml::SVM::create();
        svm->setType(cv::ml::SVM::C_SVC);
        svm->setKernel(cv::ml::SVM::RBF);
        svm->setC(1.0);
        svm->setGamma(0.1);
    }
    
    std::vector<float> extractFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
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
        
        // Feature 6: Centroid z-coordinate (indicates dent/bulge)
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        features.push_back(centroid[2]);
        
        // Feature 7-9: Normal statistics
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        ne.setRadiusSearch(0.05);
        ne.compute(*normals);
        
        float avg_curvature = 0.0f;
        for (const auto& normal : normals->points) {
            avg_curvature += normal.curvature;
        }
        avg_curvature /= normals->size();
        features.push_back(avg_curvature);
        
        // Feature 10: Standard deviation of z
        float mean_z = centroid[2];
        float var_z = 0.0f;
        for (const auto& point : cloud->points) {
            var_z += (point.z - mean_z) * (point.z - mean_z);
        }
        var_z /= cloud->size();
        features.push_back(std::sqrt(var_z));
        
        return features;
    }
    
    void loadTrainingData(const std::string& data_dir, std::vector<TrainingSample>& samples) {
        std::vector<std::string> defect_names = {"normal", "dent", "bulge", "scratch", "crack"};
        
        if (!fs::exists(data_dir)) {
            std::cerr << "Directory not found: " << data_dir << std::endl;
            return;
        }
        
        for (const auto& entry : fs::directory_iterator(data_dir)) {
            if (entry.path().extension() == ".pcd") {
                std::string filename = entry.path().filename().string();
                
                // Extract label from filename
                int label = 0;
                for (size_t i = 0; i < defect_names.size(); i++) {
                    if (filename.find(defect_names[i]) == 0) {
                        label = i;
                        break;
                    }
                }
                
                TrainingSample sample;
                sample.filename = entry.path().string();
                sample.label = label;
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(sample.filename, *cloud) == -1) {
                    std::cerr << "Failed to load: " << sample.filename << std::endl;
                    continue;
                }
                
                sample.features = extractFeatures(cloud);
                samples.push_back(sample);
                
                std::cout << "Loaded: " << sample.filename << " (label: " << defect_names[label] << ")" << std::endl;
            }
        }
    }
    
    void train(const std::vector<TrainingSample>& samples) {
        if (samples.empty()) {
            std::cerr << "No training samples!" << std::endl;
            return;
        }
        
        int num_samples = samples.size();
        int num_features = samples[0].features.size();
        
        cv::Mat training_data(num_samples, num_features, CV_32F);
        cv::Mat labels(num_samples, 1, CV_32S);
        
        for (int i = 0; i < num_samples; i++) {
            for (int j = 0; j < num_features; j++) {
                training_data.at<float>(i, j) = samples[i].features[j];
            }
            labels.at<int>(i, 0) = samples[i].label;
        }
        
        std::cout << "Training SVM with " << num_samples << " samples, " << num_features << " features..." << std::endl;
        
        cv::Ptr<cv::ml::TrainData> train_data = cv::ml::TrainData::create(training_data, cv::ml::ROW_SAMPLE, labels);
        svm->train(train_data);
        
        std::cout << "Training complete!" << std::endl;
    }
    
    void saveModel(const std::string& model_path) {
        svm->save(model_path);
        std::cout << "Model saved to: " << model_path << std::endl;
    }
    
    void loadModel(const std::string& model_path) {
        svm = cv::ml::SVM::load(model_path);
        std::cout << "Model loaded from: " << model_path << std::endl;
    }
    
    int predict(const std::vector<float>& features) {
        cv::Mat sample(1, features.size(), CV_32F);
        for (size_t i = 0; i < features.size(); i++) {
            sample.at<float>(0, i) = features[i];
        }
        
        return svm->predict(sample);
    }
    
private:
    cv::Ptr<cv::ml::SVM> svm;
};

int main(int argc, char** argv) {
    std::string data_dir = "/home/rabeb/defect_detection_ws/data/training";
    std::string model_path = "/home/rabeb/defect_detection_ws/data/svm_model.xml";
    
    SVMTrainer trainer;
    
    // Load training data
    std::vector<TrainingSample> samples;
    trainer.loadTrainingData(data_dir, samples);
    
    if (samples.empty()) {
        std::cerr << "No training data found!" << std::endl;
        return 1;
    }
    
    // Train SVM
    trainer.train(samples);
    
    // Save model
    trainer.saveModel(model_path);
    
    // Test prediction on training data
    std::cout << "\nTesting predictions on training data:" << std::endl;
    std::vector<std::string> defect_names = {"normal", "dent", "bulge", "scratch", "crack"};
    int correct = 0;
    
    for (const auto& sample : samples) {
        int predicted = trainer.predict(sample.features);
        if (predicted == sample.label) {
            correct++;
        }
        std::cout << sample.filename << ": predicted=" << defect_names[predicted] 
                  << ", actual=" << defect_names[sample.label] << std::endl;
    }
    
    float accuracy = static_cast<float>(correct) / samples.size() * 100.0f;
    std::cout << "\nTraining accuracy: " << accuracy << "%" << std::endl;
    
    return 0;
}
