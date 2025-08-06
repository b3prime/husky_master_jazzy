#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

class OccupancyGridNode : public rclcpp::Node {
	public:
		OccupancyGridNode() : Node("occupancy_grid_node") {
			// create a subscriber for the Ouster pointcloud msg
			pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
				"/ouster/points",
				rclcpp::SensorDataQoS(),
				[this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
					cloudCallback(msg);
				}
			);

			// create a publisher for the processed pointcloud msg
			pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
				"/processed_pointcloud",
				rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile()
			);
		}

	private:
		void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			// define a PCL pointcloud object and populate it with the Ouster msg
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
			pcl::fromROSMsg(*msg, *cloud);

			RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud->points.size());

			// Create a PCL VoxelGrid object and downsample the input pointcloud 
			pcl::VoxelGrid<pcl::PointXYZI> vg;
			vg.setInputCloud(cloud);
			vg.setLeafSize(0.1f, 0.1f, 0.1f);
			pcl::PointCloud<pcl::PointXYZI> downsampled_cloud;
			vg.filter(downsampled_cloud);

			// define a ROS2 pointcloud message 
			// and populate it with the downsampled PCL pointcloud
			sensor_msgs::msg::PointCloud2 out_msg;
			pcl::toROSMsg(downsampled_cloud, out_msg);
			out_msg.header = msg->header; // keep initial time and frame
			
			// publish the processed pointcloud message
			pointcloud_pub->publish(out_msg);
		}

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
};

int main(int argc, char * argv[]) {
	std::cout << "running node...";
	rclcpp::init(argc, argv);
	auto node = std::make_shared<OccupancyGridNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}


