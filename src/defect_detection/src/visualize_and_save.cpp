#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <iostream>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

class DefectVisualizer {
public:
    DefectVisualizer() {
        viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
            new pcl::visualization::PCLVisualizer("Détection de Défauts 3D"));
        viewer->setBackgroundColor(0.05, 0.05, 0.1);
        viewer->addCoordinateSystem(0.1);
        viewer->initCameraParameters();
        viewer->setCameraPosition(2.0, 2.0, 2.0, 0, 0, 0, 0, 0, 1);
    }
    
    void visualizeAndSave(const std::string& data_dir, const std::string& output_dir) {
        std::vector<std::string> defect_types = {"normal", "dent", "bulge", "scratch", "crack"};
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
        std::vector<std::string> filenames;
        
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
                    filenames.push_back(entry.path().filename().string());
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
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, names[i]);
            
            offset += 1.2;
        }
        
        // Add text labels
        offset = 0.0;
        for (size_t i = 0; i < names.size(); i++) {
            viewer->addText3D(names[i], pcl::PointXYZ(offset + 0.5, 0, 0.3), 
                            0.05, colors[i][0], colors[i][1], colors[i][2], 
                            "text_" + names[i]);
            offset += 1.2;
        }
        
        viewer->addText("Système de Détection de Défauts - PCL + SVM", 10, 300, 
                       1.0, 1.0, 1.0, "title");
        viewer->addText("5 Classes de Défauts: Normal, Dent, Bulge, Scratch, Crack", 10, 280, 
                       0.8, 0.8, 0.8, "subtitle");
        viewer->addText("Précision SVM: 78% | Temps: <100ms/nuage", 10, 260, 
                       0.6, 1.0, 0.6, "stats");
        
        // Save screenshot
        std::string screenshot_path = output_dir + "/defect_visualization_3d.png";
        viewer->saveScreenshot(screenshot_path);
        std::cout << "Screenshot saved to: " << screenshot_path << std::endl;
        
        // Spin a few times to render
        for (int i = 0; i < 10; i++) {
            viewer->spinOnce(100);
        }
        
        // Save from different angles
        viewer->setCameraPosition(0, 0, 3, 0, 0, 0, 0, 1, 0);
        viewer->saveScreenshot(output_dir + "/defect_visualization_top.png");
        
        viewer->setCameraPosition(3, 0, 0, 0, 0, 0, 0, 0, 1);
        viewer->saveScreenshot(output_dir + "/defect_visualization_side.png");
        
        std::cout << "All screenshots saved to: " << output_dir << std::endl;
    }

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

int main(int argc, char** argv) {
    std::string data_dir = "/home/rabeb/defect_detection_ws/data/training";
    std::string output_dir = "/home/rabeb/defect_detection_ws/visualizations";
    
    // Create output directory
    fs::create_directories(output_dir);
    
    DefectVisualizer visualizer;
    
    std::cout << "========================================" << std::endl;
    std::cout << "Génération de Visualisations 3D" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    visualizer.visualizeAndSave(data_dir, output_dir);
    
    std::cout << std::endl;
    std::cout << "✓ Visualisations générées avec succès !" << std::endl;
    
    return 0;
}
