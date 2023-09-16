#include "decomp_ros/visualize_polyhedron.hpp"

namespace decompros {

geometry_msgs::msg::Point toPoint(Vec3f v) {
  geometry_msgs::msg::Point p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}

VizPoly::VizPoly(const rclcpp::NodeOptions &options)
    : Node("polygon_viz", options) {

  RCLCPP_INFO(this->get_logger(), "starting!!");

  using std::placeholders::_1;

  pub_polygon_viz2_ =
      this->create_publisher<visualization_msgs::msg::Marker>("sfc/viz", 10);

  pub_polygon_array_viz2_ =
      this->create_publisher<visualization_msgs::msg::Marker>("sfc_array/viz",
                                                              10);
  
  color_r_ = this->declare_parameter<double>("color_r", color_r_);
  color_g_ = this->declare_parameter<double>("color_g", color_g_);
  color_b_ = this->declare_parameter<double>("color_b", color_b_);
  color_a_ = this->declare_parameter<double>("color_a", color_a_);

  // create the subscribers
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos_sensor_data = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  sub_polygon_ =
      this->create_subscription<decomp_ros_msgs::msg::PolyhedronStamped>(
          "sfc", qos_sensor_data, std::bind(&VizPoly::callback, this, _1));

  sub_polygon_array_ =
      this->create_subscription<decomp_ros_msgs::msg::PolyhedronArray>(
          "sfc_array", qos_sensor_data,
          std::bind(&VizPoly::array_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "Starting viz_polyhedron_component");
}

void VizPoly::callback(
    const decomp_ros_msgs::msg::PolyhedronStamped::SharedPtr msg) const {

  // convert msg to polhedron
  size_t N = msg->poly.ns.size();
  if (N == 0) {
    RCLCPP_WARN(this->get_logger(), "skipping: msg has no normals");
    return;
  }
  if (msg->poly.ps.size() != N) {
    RCLCPP_WARN(this->get_logger(), "skipping: size(msg.ns) != size(msg.ps)");
    return;
  }

  Polyhedron3D poly = convertToPolyhedron(msg->poly);

  // construct marker message
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header = msg->header;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.color.r = color_r_;
  marker_msg.color.g = color_g_;
  marker_msg.color.b = color_b_;
  marker_msg.color.a = color_a_;
  marker_msg.scale.x = 0.01;
  marker_msg.scale.y = 0.01;
  marker_msg.scale.z = 0.01;

  // add in the polygon
  add_to_marker_msg(marker_msg, poly);

  pub_polygon_viz2_->publish(marker_msg);
}

void VizPoly::array_callback(
    const decomp_ros_msgs::msg::PolyhedronArray::SharedPtr array_msg) const {

  size_t N_polys = array_msg->polys.size();
  if (N_polys == 0) {
    RCLCPP_WARN(this->get_logger(), "skipping sfc array has no polyhedrons");
    return;
  }

  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header = array_msg->header;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.color.r = color_r_;
  marker_msg.color.g = color_g_;
  marker_msg.color.b = color_b_;
  marker_msg.color.a = color_a_;
  marker_msg.scale.x = 0.01;
  marker_msg.scale.y = 0.01;
  marker_msg.scale.z = 0.01;

  for (auto msg : array_msg->polys) {

    // convert msg to polhedron
    size_t N = msg.poly.ns.size();
    if (N == 0) {
      RCLCPP_WARN(this->get_logger(), "skipping: msg has no normals");
      return;
    }
    if (msg.poly.ps.size() != N) {
      RCLCPP_WARN(this->get_logger(), "skipping: size(msg.ns) != size(msg.ps)");
      return;
    }

    auto poly = convertToPolyhedron(msg.poly);

    // add lines to marker msg
    add_to_marker_msg(marker_msg, poly);
  }

  // now publish the full msg
  pub_polygon_array_viz2_->publish(marker_msg);
}

Polyhedron3D
VizPoly::convertToPolyhedron(decomp_ros_msgs::msg::Polyhedron msg) const {

  Polyhedron3D poly;

  size_t N = msg.ns.size();
  for (size_t i = 0; i < N; ++i) {
    auto n = msg.ns[i];
    auto p = msg.ps[i];
    Hyperplane3D h(Vec3f(p.x, p.y, p.z), Vec3f(n.x, n.y, n.z));
    poly.add(h);
  }

  return poly;
}

// void VizPoly::publish_polygon_as_lines(const std_msgs::msg::Header header,
//                                        Polyhedron3D poly) const {
//
//   vec_E<vec_Vec3f> faces = cal_vertices(poly);
//   if (faces.size() == 0) {
//     RCLCPP_WARN(this->get_logger(), "Polyhedron is empty! (has 0 faces)");
//   }
//
//   // vec_E<vec_Vec3f> verts = cal_vertices(poly);
//   visualization_msgs::msg::Marker msg;
//   msg.header = header;
//   msg.type = visualization_msgs::msg::Marker::LINE_LIST;
//   msg.color.r = 1.0;
//   msg.color.a = 1.0;
//   msg.scale.x = 0.01;
//   msg.scale.y = 0.01;
//   msg.scale.z = 0.01;
//
//   // the line list will draw points in pairs
//   for (auto verts : faces) {
//
//     // each face has a list of points making up the face
//     size_t N = verts.size();
//     for (size_t i = 0; i < N; i++) {
//       msg.points.push_back(toPoint(verts[i % N]));
//       msg.points.push_back(toPoint(verts[(i + 1) % N]));
//     }
//   }
//
//   pub_polygon_viz2_->publish(msg);
// }

void VizPoly::add_to_marker_msg(visualization_msgs::msg::Marker &marker_msg,
                                Polyhedron3D &poly) const {

  vec_E<vec_Vec3f> faces = cal_vertices(poly);
  if (faces.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Polyhedron is empty! (has 0 faces)");
  }

  // the line list will draw points in pairs
  for (auto verts : faces) {

    // each face has a list of points making up the face
    size_t N = verts.size();
    if (faces.size() == 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Polyhedron face is empty! (has 0 verts)");
    }
    for (size_t i = 0; i < N; i++) {
      marker_msg.points.push_back(toPoint(verts[i % N]));
      marker_msg.points.push_back(toPoint(verts[(i + 1) % N]));
    }
  }
}

} // namespace decompros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decompros::VizPoly)
