
#ifndef DECOMPROS
#define DECOMPROS

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <decomp_util/seed_decomp.h>


namespace decompros
{


	using sensor_msgs::msg::PointCloud2;
	using PCLPoint = pcl::PointXYZ;
	using PCLPointCloud = pcl::PointCloud<PCLPoint>;

	class DecompROS : public rclcpp::Node 
	{
		public:

			//using PCLPoint = pcl::PointXYZ;
                        //using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;

			//explicit DecompROS(const rclcpp::NodeOptions & options);
			DecompROS();

			void cloud_callback(const PointCloud2::SharedPtr msg) const;

		protected:
			rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_sub_;
			rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_publisher_;
			rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr debug_publisher_;

			bool publish_debug_pc_ = false;

	}; // class DecompROS


} // namespace decompros


#endif // DECOMPROS
