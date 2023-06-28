#ifndef DECOMPROS_SEEDDECOMP
#define DECOMPROS_SEEDDECOMP

#include "decomp_ros_msgs/msg/polyhedron_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include <decomp_util/line_segment.h>
#include <decomp_util/seed_decomp.h>

namespace decompros {
using sensor_msgs::msg::PointCloud2;
using PCLPoint = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;

class SeedDecomp : public rclcpp::Node {

public:
  explicit SeedDecomp(const rclcpp::NodeOptions &options);

protected:
  // subs
  rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_sub_;

  // pubs
  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronStamped>::SharedPtr
      pub_polygon_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_fov_;

  // params
  bool publish_fov_pc_ = false;
  double fov_h_ = 180.0;           // deg
  double fov_v_ = 180.0;           // deg
  double fov_obs_spacing_ = 0.25;  // m
  double fov_range_ = 5.0;         // m
  double fov_obs_ray_range_ = 5.0; // m
  int fov_obs_skip_first_ = 3;

  // vars
  vec_Vec3f fov_obs_;

  // methods

  void add_fov_ray(double alt, double azi);
  void initialize_fov_obs();
  void publish_fov_obs();
  void publish_fov_obs(std_msgs::msg::Header);
  void cloud_callback(const PointCloud2::SharedPtr msg) const;
  void publish_polyhedron(std_msgs::msg::Header, Polyhedron3D) const;
};

} // namespace decompros

#endif // DECOMPROS_SEEDDECOMP
