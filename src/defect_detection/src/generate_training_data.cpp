#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <random>
#include <iostream>
#include <fstream>

void generatePlaneWithDefects(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                              int defect_type, double defect_intensity) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise(0.0, 0.001);
    
    // Generate base plane
    for (float x = -0.5; x <= 0.5; x += 0.01) {
        for (float y = -0.5; y <= 0.5; y += 0.01) {
            pcl::PointXYZ p;
            p.x = x + noise(gen);
            p.y = y + noise(gen);
            p.z = noise(gen);
            
            // Add defect based on type
            if (defect_type == 1) { // Dent
                if (x > 0.1 && x < 0.2 && y > 0.1 && y < 0.2) {
                    p.z -= defect_intensity;
                }
            } else if (defect_type == 2) { // Bulge
                if (x > 0.1 && x < 0.2 && y > 0.1 && y < 0.2) {
                    p.z += defect_intensity;
                }
            } else if (defect_type == 3) { // Scratch
                if (x > -0.3 && x < -0.1 && std::abs(y - (-0.3 + (x + 0.3) * 0.5)) < 0.01) {
                    p.z += defect_intensity * 0.5;
                }
            } else if (defect_type == 4) { // Crack
                if (x > 0.2 && x < 0.3 && y > 0.2 && y < 0.3) {
                    p.z -= defect_intensity * (1.0 + noise(gen));
                }
            }
            
            cloud->push_back(p);
        }
    }
}

void preprocessCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Statistical outlier removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);
    
    // Voxel grid downsampling
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(0.01, 0.01, 0.01);
    vox.filter(*cloud);
}

int main(int argc, char** argv) {
    // Generate training data for different defect types
    std::vector<std::string> defect_names = {"normal", "dent", "bulge", "scratch", "crack"};
    
    for (int defect_type = 0; defect_type < 5; defect_type++) {
        for (int sample = 0; sample < 10; sample++) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            double intensity = 0.02 + (sample * 0.005);
            generatePlaneWithDefects(cloud, defect_type, intensity);
            preprocessCloud(cloud);
            
            std::string filename = "/home/rabeb/defect_detection_ws/data/training/" + 
                                   defect_names[defect_type] + "_" + 
                                   std::to_string(sample) + ".pcd";
            
            pcl::io::savePCDFileBinary(filename, *cloud);
            std::cout << "Generated: " << filename << " (" << cloud->size() << " points)" << std::endl;
        }
    }
    
    std::cout << "Training data generation complete!" << std::endl;
    return 0;
}
