#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <input.pcd> <output_ascii.pcd>" << std::endl;
        return 1;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Load binary PCD
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
        std::cerr << "Failed to load: " << argv[1] << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << cloud->size() << " points" << std::endl;
    
    // Save as ASCII
    pcl::io::savePCDFileASCII(argv[2], *cloud);
    
    std::cout << "Saved to: " << argv[2] << std::endl;
    return 0;
}
