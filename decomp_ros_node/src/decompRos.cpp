
#include <decompRos/decompRos.hpp>
#include <decomp_geometry/geometric_utils.h>

namespace decompros {

// 	DecompROS::DecompROS(const rclcpp::NodeOptions & node_options)
// 		: Node("decompROS", node_options)
DecompROS::DecompROS() : Node("decompROS") {

  RCLCPP_INFO(this->get_logger(), "Started SFC node");

  using std::placeholders::_1;
  // create publishers
  polygon_publisher_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          "safe_flight_cooridor_polygon", 10);
  debug_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud>("fov_obs", 10);


  // parameters
  this->declare_parameter<bool>("publish_debug_pc", false);
  publish_debug_pc_ = this->get_parameter("publish_debug_pc").as_bool();

  // create subscribers
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                         qos_profile);

  point_cloud_sub_ = this->create_subscription<PointCloud2>(
      "cloud_in", qos, std::bind(&DecompROS::cloud_callback, this, _1));
}

void DecompROS::cloud_callback(const PointCloud2::SharedPtr msg) const {

  auto start = std::chrono::high_resolution_clock::now();

  // RCLCPP_DEBUG(this->get_logger(), "I got a pc with %u width and %u height",
  // msg->width, msg->height);

  PCLPointCloud pc;
  pcl::fromROSMsg(*msg, pc);

  auto now = std::chrono::high_resolution_clock::now();

  auto dur = std::chrono::duration_cast<std::chrono::microseconds>(now - start);

  RCLCPP_DEBUG(this->get_logger(), "pcl pc has %zu points and took %lu us",
              pc.size(), dur.count());

  // ADD ALL THE FOV OBSTACLES
  vec_Vec3f obs;
  vec_Vec3f fov_obs;
  for (PCLPointCloud::const_iterator it = pc.begin(); it != pc.end(); ++it) {
    obs.push_back(Vec3f(it->x, it->y, it->z));
  }

  // ADD ALL THE VISUAL CONE OBSTACLES
  const Vec3f origin(0, 0, 0);

  bool add_fov_constraints = true;

  if (add_fov_constraints) {
    double fov_h = 87.0 * M_PI / 180.0;
    double fov_v = 58.0 * M_PI / 180.0;
    double spacing = 0.25;




    Vec3f dir_up_right(sin(fov_h / 2) * cos(fov_v / 2), -sin(fov_v / 2),
                       cos(fov_h / 2) * cos(fov_v / 2));
    Vec3f dir_up_left(sin(-fov_h / 2) * cos(fov_v / 2), -sin(fov_v / 2),
                      cos(-fov_h / 2) * cos(fov_v / 2));
    Vec3f dir_down_left(sin(-fov_h / 2) * cos(-fov_v / 2), -sin(-fov_v / 2),
                        cos(-fov_h / 2) * cos(-fov_v / 2));
    Vec3f dir_down_right(sin(fov_h / 2) * cos(-fov_v / 2), -sin(-fov_v / 2),
                         cos(fov_h / 2) * cos(-fov_v / 2));
    Vec3f dir_down(
		    0,
		    -sin(fov_v/2),
		    cos(fov_v/2));
    Vec3f dir_up(
		    0,
		    sin(fov_v/2),
		    cos(fov_v/2));
    Vec3f dir_right(sin(fov_h/2), 0, cos(fov_h/2));
    Vec3f dir_left(-sin(fov_h/2), 0, cos(fov_h/2));


    for (auto v : {
		    dir_up, dir_down, dir_right, dir_left,
		    dir_up_right, dir_up_left, dir_down_left, dir_down_right
		    }){
    for (int i = -1; i < 25; i++) {
      if ((i >= 0) && (i < 3)) {
        continue;
      }
      Vec3f p = origin + v * spacing * i;
      obs.push_back(p);
      fov_obs.push_back(p);
    }
    }


  now = std::chrono::high_resolution_clock::now();
  dur = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
  RCLCPP_DEBUG(this->get_logger(), "make obs list: %lu us", dur.count());

  // now actually construct the sfc
  SeedDecomp3D decomp(origin + Vec3f(0, 0, 0.5));
  decomp.set_obs(obs);
  decomp.set_local_bbox(Vec3f(5, 5, 5));
  decomp.dilate(0.05);

  auto poly = decomp.get_polyhedron();

  RCLCPP_DEBUG(this->get_logger(), "decomp worked");

  now = std::chrono::high_resolution_clock::now();

  dur = std::chrono::duration_cast<std::chrono::microseconds>(now - start);

  RCLCPP_DEBUG(this->get_logger(), "fin: %lu", dur.count());

  // now we try and visualize it in rviz
  // lets see how we can construct the polyhedron

  vec_E<vec_Vec3f> verts = cal_vertices(poly);

  // ok seems reasonable, lets try creating a polygon - need to publish
  // geometry_msgs::msgs::PolygonStampled
  geometry_msgs::msg::PolygonStamped msg_poly;
  msg_poly.header.stamp = msg->header.stamp;
  msg_poly.header.frame_id =
      msg->header.frame_id; // use the original pointclouds frame id
  // now start pushing the points
  for (size_t i = 0; i < verts.size(); ++i) {
    auto vs = verts[i];
    for (size_t j = 0; j < vs.size(); ++j) {
      auto v = vs[j];
      geometry_msgs::msg::Point32 p;
      p.x = v(0);
      p.y = v(1);
      p.z = v(2);
      msg_poly.polygon.points.push_back(p);
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "msg_poly has %zu points",
              msg_poly.polygon.points.size());

  // publish the polygon
  polygon_publisher_->publish(msg_poly);
    
  // plot the points for debugging purposes
    //
    if (publish_debug_pc_) {
      sensor_msgs::msg::PointCloud debug_msg;
      for (size_t i = 0; i < obs.size(); ++i) {
        Vec3f v = obs[i];
        geometry_msgs::msg::Point32 p;
        p.x = v(0);
        p.y = v(1);
        p.z = v(2);
        debug_msg.points.push_back(p);
      }
      debug_msg.header.stamp = msg->header.stamp;
      debug_msg.header.frame_id = msg->header.frame_id;
      debug_publisher_->publish(debug_msg);
    }
  }
}

} // namespace decompros

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<decompros::DecompROS>());
  rclcpp::shutdown();
  return 0;
}

// // register it as a component
// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(decompros::DecompROS)
