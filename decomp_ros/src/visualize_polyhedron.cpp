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
      this->create_publisher<visualization_msgs::msg::Marker>("sfc/viz", 1);

  pub_polygon_array_viz2_ =
      this->create_publisher<visualization_msgs::msg::Marker>("sfc_array/viz",
                                                              1);

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
  marker_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker_msg.color.r = color_r_;
  marker_msg.color.g = color_g_;
  marker_msg.color.b = color_b_;
  marker_msg.color.a = color_a_;
  marker_msg.scale.x = 1;
  marker_msg.scale.y = 1;
  marker_msg.scale.z = 1;

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
  marker_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker_msg.color.r = color_r_;
  marker_msg.color.g = color_g_;
  marker_msg.color.b = color_b_;
  marker_msg.color.a = color_a_;
  marker_msg.scale.x = 1.0;
  marker_msg.scale.y = 1.0;
  marker_msg.scale.z = 1.0;

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

// void VizPoly::add_to_marker_msg(visualization_msgs::msg::Marker &marker_msg,
//                                 Polyhedron3D &poly) const {
//
//   vec_E<vec_Vec3f> faces = cal_vertices(poly);
//   if (faces.size() == 0) {
//     RCLCPP_WARN(this->get_logger(), "Polyhedron is empty! (has 0 faces)");
//   }
//
//   // the line list will draw points in pairs
//   for (auto verts : faces) {
//
//     // each face has a list of points making up the face
//     size_t N = verts.size();
//     if (faces.size() == 0) {
//       RCLCPP_WARN(this->get_logger(),
//                   "Polyhedron face is empty! (has 0 verts)");
//     }
//     for (size_t i = 0; i < N; i++) {
//       marker_msg.points.push_back(toPoint(verts[i % N]));
//       marker_msg.points.push_back(toPoint(verts[(i + 1) % N]));
//     }
//   }
// }

void VizPoly::add_to_marker_msg(visualization_msgs::msg::Marker &marker_msg,
                                Polyhedron3D &poly) const {

  // this version tries to plot a triangle list

  vec_E<vec_Vec3f> faces = cal_vertices(poly);
  if (faces.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Polyhedron is empty! (has 0 faces)");
  }

  for (auto verts : faces) {
    // each face has a list of points making up the face
    size_t N = verts.size();
    if (N < 3) {
      RCLCPP_WARN(this->get_logger(),
                  "Polyhedron face is empty! (has < 3 verts)");
    }
    for (size_t i = 1; i < N - 1; i++) {
      marker_msg.points.push_back(toPoint(verts[0])); // form the triangle
      marker_msg.points.push_back(toPoint(verts[i]));
      marker_msg.points.push_back(toPoint(verts[i + 1]));
    }
  }
}

} // namespace decompros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decompros::VizPoly)
