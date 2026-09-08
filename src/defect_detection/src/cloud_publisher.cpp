#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

class CloudPublisher : public rclcpp::Node {
public:
    CloudPublisher() : Node("cloud_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/input_cloud", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CloudPublisher::publishCloud, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "CloudPublisher node started");
    }

private:
    void publishCloud() {
        // Generate synthetic point cloud with simulated defects
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Generate base plane
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> noise(0.0, 0.001);
        
        for (float x = -0.5; x <= 0.5; x += 0.01) {
            for (float y = -0.5; y <= 0.5; y += 0.01) {
                pcl::PointXYZ p;
                p.x = x + noise(gen);
                p.y = y + noise(gen);
                p.z = noise(gen);
                cloud->push_back(p);
            }
        }
        
        // Add simulated defect (dent)
        for (float x = 0.1; x <= 0.2; x += 0.005) {
            for (float y = 0.1; y <= 0.2; y += 0.005) {
                pcl::PointXYZ p;
                p.x = x + noise(gen);
                p.y = y + noise(gen);
                p.z = -0.02 + noise(gen); // Dent depth
                cloud->push_back(p);
            }
        }
        
        // Add simulated defect (scratch)
        for (float t = 0.0; t <= 1.0; t += 0.01) {
            pcl::PointXYZ p;
            p.x = -0.3 + t * 0.2 + noise(gen);
            p.y = -0.3 + t * 0.1 + noise(gen);
            p.z = 0.005 + noise(gen); // Scratch height
            cloud->push_back(p);
        }
        
        // Convert to ROS2 message
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = "base_link";
        msg.header.stamp = this->now();
        
        publisher_->publish(msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published cloud with %zu points", cloud->size());
    }
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
