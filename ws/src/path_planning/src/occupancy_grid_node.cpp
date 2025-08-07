#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

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

			// create a publisher for the occupancy grid msg
			grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
				"/occupancy_grid",
				rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile()
			);
		}

	private:
		void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
			// define a PCL pointcloud object and populate it with the Ouster msg
			using CloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

			CloudXYZI::Ptr raw_cloud (new CloudXYZI);
			CloudXYZI::Ptr downsampled_cloud (new CloudXYZI);
			CloudXYZI::Ptr final_cloud (new CloudXYZI);

			float cell_size = 0.05f; // of the downsampling / occupancy map
			float x_bound = 5.0f; // [-x_bound, x_bound] is kept
			float y_bound = 5.0f; // [-y_bound, y_bound] is kept
			float z_bound = 0.2f; // the upper bound on the z-axis. min bound is -0.2m

			// Retrieve the raw cloud from ros2 topic
			pcl::fromROSMsg(*msg, *raw_cloud);
			RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", raw_cloud->points.size());
			
			pcl::VoxelGrid<pcl::PointXYZI> vg;
			vg.setInputCloud(raw_cloud);
			vg.setLeafSize(cell_size, cell_size, cell_size);
			vg.filter(*downsampled_cloud);

			// Keep only -10m...10m in x and y and -0.2m...0.2m in z
			pcl::CropBox<pcl::PointXYZI> box;
			box.setInputCloud (downsampled_cloud);
			box.setMin (Eigen::Vector4f(-x_bound, -y_bound, -0.2f, 1.0f));
			box.setMax (Eigen::Vector4f( x_bound, y_bound, z_bound, 1.0f));
			box.setNegative (false);
			box.filter( *final_cloud);

			// convert the downsampled and cropped pointcloud to an occupancy map
			const int width  = static_cast<int>(std::ceil(x_bound * 2 / cell_size));
			const int height = static_cast<int>(std::ceil(y_bound * 2 / cell_size));
			
			std::vector<int8_t> grid(width * height, 0);
	
			// loop through all points, mark its index in the grid.
			for (const auto& p : final_cloud->points)
			{
				// find the x and y "indices" if we were using a 2d array for this
				int ix = static_cast<int>(std::floor((p.x - (-x_bound)) / cell_size));
				int iy = static_cast<int>(std::floor((p.y - (-y_bound)) / cell_size));

				grid[iy * width + ix] = 100;
			}

			// define a ROS2 pointcloud message 
			// and populate it with the downsampled PCL pointcloud
			sensor_msgs::msg::PointCloud2 out_msg;
			pcl::toROSMsg(*final_cloud, out_msg);
			out_msg.header = msg->header; // keep initial time and frame
			// publish the pointcloud
			pointcloud_pub->publish(out_msg);


			nav_msgs::msg::OccupancyGrid occ;
			occ.header.frame_id = "map";
			occ.info.resolution = cell_size;
			occ.info.width = width;
			occ.info.height = height;
			occ.info.origin.position.x = -x_bound;
			occ.info.origin.position.y = -y_bound;
			occ.data = std::move(grid);
			// publish the occupancy grid
			grid_pub->publish(occ);
		}

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub;
};

int main(int argc, char * argv[]) {
	std::cout << "running node...";
	rclcpp::init(argc, argv);
	auto node = std::make_shared<OccupancyGridNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}


