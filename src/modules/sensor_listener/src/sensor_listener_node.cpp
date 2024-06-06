#include "sensor_listener_node.hpp"

SensorsReader::SensorsReader() : Node("sensor_reader_node")
{
  point_cloud_sub = this->create_subscription<pointCloudMsg>("lidar", 100,
                                                             std::bind(&SensorsReader::calbackPointCloud, this, _1));
  imu_sub = this->create_subscription<imuMsg>("imu", 10,
                                              std::bind(&SensorsReader::calbackImu, this, _1));
  point_cloud_pub = this->create_publisher<pointCloudMsg>("merge_cloud", 100);
}

void SensorsReader::calbackPointCloud(pointCloudMsg msg)
{
  cloud_data = msg;
  cloud_data.header.frame_id = "vehicle";
  std::vector<uint8_t> &data = msg.data;
  for (size_t row = 0; row < msg.height; ++row)
  {
    for (size_t col = 0; col < msg.width; ++col)
    {
      uint32_t index = row * msg.row_step + col * msg.point_step;
      float x = *(reinterpret_cast<float *>(&data[index]));
      float y = *(reinterpret_cast<float *>(&data[index + 4]));
      float z = *(reinterpret_cast<float *>(&data[index + 8]));
      RCLCPP_DEBUG(this->get_logger(), "msg[%ld][%ld] = x: %0.2f - y: %0.2f - z: %0.2f", row, col, x, y, z);
    }
  }
  point_cloud_pub->publish(cloud_data);
}

void SensorsReader::calbackImu(imuMsg msg)
{
  imu_data = msg;
}

int main(int argc, char **args)
{
  rclcpp::init(argc, args);
  rclcpp::spin(std::make_shared<SensorsReader>());
  rclcpp::shutdown();
}