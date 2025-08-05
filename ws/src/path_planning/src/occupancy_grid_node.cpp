#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class OccupancyGridNode : public rclcpp::Node {
	public:
		OccupancyGridNode() : Node("occupancy_grid_node") {
			pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
				"/ouster/points", 10,
				[this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
					cloudCallback(msg);
				}
			);
		}

	private:
		void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			pcl::PointCloud<pcl::PointXYZI> cloud;
			pcl::fromROSMsg(*msg, cloud);

			RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud.points.size());

			for (const auto& pt : cloud.points) {
				std::cout << "x: " << pt.x << ", y: " << pt.y << ", z: " << pt.z << std::endl;
			}
		}

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_;
};

int main(int argc, char * argv[]) {
	std::cout << "running node...";
	rclcpp::init(argc, argv);
	auto node = std::make_shared<OccupancyGridNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
