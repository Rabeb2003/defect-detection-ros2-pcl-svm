#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <filesystem>
#include <vector>
#include <thread>
#include <chrono>

namespace fs = std::filesystem;

class DefectVisualizer {
public:
    DefectVisualizer() {
        viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
            new pcl::visualization::PCLVisualizer("Détection de Défauts 3D"));
        viewer->setBackgroundColor(0.1, 0.1, 0.1);
        viewer->addCoordinateSystem(0.1);
        viewer->initCameraParameters();
    }
    
    void visualizeDataset(const std::string& data_dir) {
        std::vector<std::string> defect_types = {"normal", "dent", "bulge", "scratch", "crack"};
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
        
        // Load one sample from each defect type
        for (const auto& defect : defect_types) {
            for (const auto& entry : fs::directory_iterator(data_dir)) {
                if (entry.path().extension() == ".pcd" && 
                    entry.path().filename().string().find(defect) == 0) {
                    
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    if (pcl::io::loadPCDFile<pcl::PointXYZ>(entry.path().string(), *cloud) == -1) {
                        continue;
                    }
                    
                    clouds.push_back(cloud);
                    std::cout << "Loaded: " << entry.path().filename().string() 
                              << " (" << cloud->size() << " points)" << std::endl;
                    break;
                }
            }
        }
        
        // Visualize each cloud with different colors
        std::vector<std::vector<float>> colors = {
            {0.0, 1.0, 0.0},  // Green - Normal
            {1.0, 0.0, 0.0},  // Red - Dent
            {1.0, 0.5, 0.0},  // Orange - Bulge
            {1.0, 1.0, 0.0},  // Yellow - Scratch
            {0.5, 0.0, 1.0}   // Purple - Crack
        };
        
        std::vector<std::string> names = {"Normal", "Dent", "Bulge", "Scratch", "Crack"};
        
        float offset = 0.0;
        for (size_t i = 0; i < clouds.size(); i++) {
            // Offset clouds for better visualization
            for (auto& point : clouds[i]->points) {
                point.x += offset;
            }
            
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
                color_handler(clouds[i], colors[i][0], colors[i][1], colors[i][2]);
            
            viewer->addPointCloud<pcl::PointXYZ>(clouds[i], color_handler, names[i]);
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, names[i]);
            
            offset += 1.2;
        }
        
        // Add text labels
        offset = 0.0;
        for (size_t i = 0; i < names.size(); i++) {
            viewer->addText(names[i], 10, 10 + i * 30, 
                           colors[i][0], colors[i][1], colors[i][2], 
                           "text_" + names[i]);
            offset += 1.2;
        }
        
        viewer->addText("Système de Détection de Défauts - PCL + SVM", 10, 300, 
                       1.0, 1.0, 1.0, "title");
        
        std::cout << "\nVisualisation 3D interactive..." << std::endl;
        std::cout << "Utilisez la souris pour : " << std::endl;
        std::cout << "- Clic gauche : Rotation" << std::endl;
        std::cout << "- Clic droit : Zoom" << std::endl;
        std::cout << "- Clic molette : Pan" << std::endl;
        std::cout << "- 'q' ou 'ESC' : Quitter" << std::endl;
        
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    void visualizeSingleDefect(const std::string& pcd_file, const std::string& defect_type) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
            std::cerr << "Failed to load: " << pcd_file << std::endl;
            return;
        }
        
        std::cout << "Visualizing: " << pcd_file << " (" << cloud->size() << " points)" << std::endl;
        
        // Color based on Z value (height)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        
        float min_z = cloud->points[0].z, max_z = cloud->points[0].z;
        for (const auto& p : cloud->points) {
            min_z = std::min(min_z, p.z);
            max_z = std::max(max_z, p.z);
        }
        
        for (const auto& p : cloud->points) {
            pcl::PointXYZRGB point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            
            // Color based on Z (defects will be different colors)
            float normalized_z = (p.z - min_z) / (max_z - min_z + 0.001);
            point.r = static_cast<uint8_t>(normalized_z * 255);
            point.g = static_cast<uint8_t>((1.0 - normalized_z) * 255);
            point.b = 128;
            
            colored_cloud->push_back(point);
        }
        
        viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, defect_type);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, defect_type);
        
        viewer->addText("Défaut: " + defect_type, 10, 10, 1.0, 1.0, 1.0, "defect_label");
        viewer->addText("Couleur = hauteur (Z)", 10, 40, 1.0, 1.0, 1.0, "legend");
        
        std::cout << "Press 'q' to continue..." << std::endl;
        
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        viewer->removePointCloud(defect_type);
        viewer->removeShape("defect_label");
        viewer->removeShape("legend");
        viewer->resetStoppedFlag();
    }

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

int main(int argc, char** argv) {
    std::string data_dir = "/home/rabeb/defect_detection_ws/data/training";
    
    DefectVisualizer visualizer;
    
    std::cout << "========================================" << std::endl;
    std::cout << "Visualiseur 3D de Défauts" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // Visualize all defect types side by side
    std::cout << "Mode 1: Visualisation comparative des 5 types de défauts" << std::endl;
    visualizer.visualizeDataset(data_dir);
    
    return 0;
}
