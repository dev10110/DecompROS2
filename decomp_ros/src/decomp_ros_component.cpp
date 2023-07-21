#include "decomp_ros/decomp_ros_component.hpp"
#include <decomp_geometry/geometric_utils.h>
#include <iostream>
#include <memory>

namespace decompros {

SeedDecomp::SeedDecomp(const rclcpp::NodeOptions &options)
    : Node("seed_decomp", options) {

  using std::placeholders::_1;

  // create the publishers
  pub_polygon_ =
      this->create_publisher<decomp_ros_msgs::msg::PolyhedronStamped>("sfc",
                                                                      10);
  pub_fov_ =
      this->create_publisher<sensor_msgs::msg::PointCloud>("fov_obstacles", 10);

  // create the subscribers
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos_sensor_data = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  point_cloud_sub_ = this->create_subscription<PointCloud2>(
      "cloud_in", qos_sensor_data,
      std::bind(&SeedDecomp::cloud_callback, this, _1));

  // create the parameters
  this->declare_parameter<bool>("publish_fov_obstacles", false);
  publish_fov_pc_ = this->get_parameter("publish_fov_obstacles").as_bool();

  this->declare_parameter<double>("fov_h", 87.0);
  fov_h_ = this->get_parameter("fov_h").as_double();

  this->declare_parameter<double>("fov_v", 58.0);
  fov_v_ = this->get_parameter("fov_v").as_double();

  this->declare_parameter<double>("fov_range", 5.0);
  fov_range_ = this->get_parameter("fov_range").as_double();

  this->declare_parameter<double>("fov_obs_ray_range", 5.0);
  fov_obs_ray_range_ = this->get_parameter("fov_obs_ray_range").as_double();

  this->declare_parameter<double>("fov_obs_spacing", 0.5);
  fov_obs_spacing_ = this->get_parameter("fov_obs_spacing").as_double();

  this->declare_parameter<int>("fov_obs_skip_first", 3);
  fov_obs_skip_first_ = this->get_parameter("fov_obs_skip_first").as_int();

  // initialize fov_obs
  initialize_fov_obs();

  RCLCPP_INFO(this->get_logger(), "Starting seedDecomp_component");
}

void SeedDecomp::add_fov_ray(double alt, double azi) {

  Vec3f origin(0, 0, 0);

  Vec3f dir(sin(azi) * cos(alt), -sin(alt), cos(azi) * cos(alt));

  int N = fov_obs_ray_range_ / fov_obs_spacing_;

  for (int i = -1; i < N; i++) {
    if ((i >= 0) && (i < fov_obs_skip_first_)) {
      continue;
    }
    Vec3f p = origin + dir * fov_obs_spacing_ * i;
    fov_obs_.push_back(p);
  }

  return;
}

void SeedDecomp::initialize_fov_obs() {

  double alt = fov_v_ / 2 * (M_PI / 180.0);
  double azi = fov_h_ / 2 * (M_PI / 180.0);

  int N = 3;

  for (int i = -N; i <= N; i++) {

    double f = double(i) / double(N);

    add_fov_ray(alt, f * azi);
    add_fov_ray(-alt, f * azi);

    add_fov_ray(f * alt, azi);
    add_fov_ray(f * alt, -azi);
  }

  publish_fov_obs();

  return;
}

void SeedDecomp::publish_fov_obs() {

  std_msgs::msg::Header header;
  header.frame_id = "camera_depth_optical_frame";
  header.stamp = this->get_clock()->now();

  publish_fov_obs(header);
}

void SeedDecomp::publish_fov_obs(std_msgs::msg::Header header) {

  sensor_msgs::msg::PointCloud msg;
  for (size_t i = 0; i < fov_obs_.size(); ++i) {

    for (Vec3f v : fov_obs_) {
      geometry_msgs::msg::Point32 p;
      p.x = v(0);
      p.y = v(1);
      p.z = v(2);
      msg.points.push_back(p);
    }
    msg.header = header;
    pub_fov_->publish(msg);
  }
}

void SeedDecomp::cloud_callback(const PointCloud2::SharedPtr msg) const {

  auto start = std::chrono::high_resolution_clock::now();

  PCLPointCloud pc;
  pcl::fromROSMsg(*msg, pc);

  auto now = std::chrono::high_resolution_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
  RCLCPP_DEBUG(this->get_logger(), "pcl pc has %zu points and took %lu us",
               pc.size(), dur.count());

  // ADD ALL THE OBSTACLES
  vec_Vec3f obs;
  for (PCLPointCloud::const_iterator it = pc.begin(); it != pc.end(); ++it) {
    obs.push_back(Vec3f(it->x, it->y, it->z));
  }

  // Add in the fov obs
  for (auto v : fov_obs_) {
    obs.push_back(v);
  }
  now = std::chrono::high_resolution_clock::now();
  dur = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
  RCLCPP_DEBUG(this->get_logger(), "make obs list: %lu us", dur.count());

  // now actually construct the sfc
  SeedDecomp3D decomp(Vec3f(0, 0, 0.25));
  // LineSegment3D decomp(Vec3f(0,0,0), Vec3f(0, 0, 0.25));
  decomp.set_obs(obs);
  decomp.set_local_bbox(Vec3f(2 * fov_range_, 2 * fov_range_, 2 * fov_range_));
  decomp.dilate(0.05);

  auto poly = decomp.get_polyhedron();

  RCLCPP_DEBUG(this->get_logger(), "decomp worked");

  now = std::chrono::high_resolution_clock::now();
  dur = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
  RCLCPP_DEBUG(this->get_logger(), "fin: %lu", dur.count());

  // publish results
  publish_polyhedron(msg->header, poly);
}

void SeedDecomp::publish_polyhedron(const std_msgs::msg::Header header,
                                    Polyhedron3D poly) const {

  decomp_ros_msgs::msg::PolyhedronStamped msg;
  msg.header = header;

  for (const auto &hp : poly.hyperplanes()) {
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Vector3 normal;

    point.x = hp.p_(0);
    point.y = hp.p_(1);
    point.z = hp.p_(2);
    normal.x = hp.n_(0);
    normal.y = hp.n_(1);
    normal.z = hp.n_(2);

    msg.poly.ps.push_back(point);
    msg.poly.ns.push_back(normal);
  }

  pub_polygon_->publish(msg);
}

} // namespace decompros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decompros::SeedDecomp)
