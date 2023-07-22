#include "sensors_reader.hpp"

SensorsReader::SensorsReader(): Node("sensor_reader_node"){
  point_cloud_sub = this->create_subscription<pointCloudMsg>("lidar", 100,
                    std::bind(&SensorsReader::calbackPointCloud, this, _1));
  imu_sub = this->create_subscription<imuMsg>("imu", 100, 
            std::bind(&SensorsReader::calbackImu, this, _1));
  point_cloud_pub = this->create_publisher<pointCloudMsg>("merge_cloud", 100); 
}

void SensorsReader::calbackPointCloud(pointCloudMsg msg){
  cloud_data = msg;
  cloud_data.header.frame_id = "vehicle";
  point_cloud_pub->publish(cloud_data);
}

void SensorsReader::calbackImu(imuMsg msg){
  imu_data = msg;
}

int main(int argc, char ** args){
    rclcpp::init(argc, args);
    rclcpp::spin(std::make_shared<SensorsReader>());
    rclcpp::shutdown();
}