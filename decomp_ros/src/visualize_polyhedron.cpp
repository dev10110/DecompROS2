#include "decomp_ros/visualize_polyhedron.hpp"
#include <decomp_geometry/geometric_utils.h>
#include <iostream>
#include <memory>

namespace decompros {

VizPoly::VizPoly(const rclcpp::NodeOptions &options)
    : Node("polygon_viz", options) {

  RCLCPP_INFO(this->get_logger(), "starting!!");

  using std::placeholders::_1;

  // create the publishers
  // pub_polygon_viz_ =
  // this->create_publisher<geometry_msgs::msg::PolygonStamped>(
  //     "sfc_viz", 10);
  pub_polygon_viz2_ =
      this->create_publisher<visualization_msgs::msg::Marker>("sfc/viz", 10);

  // create the subscribers
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos_sensor_data = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  sub_polygon_ =
      this->create_subscription<decomp_ros_msgs::msg::PolyhedronStamped>(
          "sfc", qos_sensor_data, std::bind(&VizPoly::callback, this, _1));

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

  Polyhedron3D poly;
  // construct the polygon
  for (size_t i = 0; i < N; ++i) {
    auto n = msg->poly.ns[i];
    auto p = msg->poly.ps[i];
    // get the hyperplane
    Hyperplane3D h(Vec3f(p.x, p.y, p.z), Vec3f(n.x, n.y, n.z));

    // add to polyhedron
    poly.add(h);
  }

  // publish as polygons
  publish_polygon_as_lines(msg->header, poly);
  // to debug, might be helpful to plot this too
  // publish_polygon_stamped(msg->header, poly);
}

geometry_msgs::msg::Point toPoint(Vec3f v) {
  geometry_msgs::msg::Point p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}

void VizPoly::publish_polygon_as_lines(const std_msgs::msg::Header header,
                                       Polyhedron3D poly) const {

  vec_E<vec_Vec3f> faces = cal_vertices(poly);
  if (faces.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Polyhedron is empty! (has 0 faces)");
  }

  // vec_E<vec_Vec3f> verts = cal_vertices(poly);
  visualization_msgs::msg::Marker msg;
  msg.header = header;
  msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  msg.color.r = 1.0;
  msg.color.a = 1.0;
  msg.scale.x = 0.01;
  msg.scale.y = 0.01;
  msg.scale.z = 0.01;

  // the line list will draw points in pairs
  for (auto verts : faces) {

    // each face has a list of points making up the face
    size_t N = verts.size();
    for (size_t i = 0; i < N; i++) {
      msg.points.push_back(toPoint(verts[i % N]));
      msg.points.push_back(toPoint(verts[(i + 1) % N]));
    }
  }

  pub_polygon_viz2_->publish(msg);
}

// void VizPoly::publish_polygon_stamped(const std_msgs::msg::Header header,
//                                          Polyhedron3D poly) const {
// 
//   vec_E<vec_Vec3f> verts = cal_vertices(poly);
// 
//   geometry_msgs::msg::PolygonStamped msg_poly;
//   msg_poly.header = header;
// 
//   // now start pushing the points
//   for (size_t i = 0; i < verts.size(); ++i) {
//     auto vs = verts[i];
//     for (size_t j = 0; j < vs.size(); ++j) {
//       auto v = vs[j];
//       geometry_msgs::msg::Point32 p;
//       p.x = v(0);
//       p.y = v(1);
//       p.z = v(2);
//       msg_poly.polygon.points.push_back(p);
//     }
//   }
//   RCLCPP_DEBUG(this->get_logger(), "msg_poly has %zu points",
//                msg_poly.polygon.points.size());
// 
//   // publish the polygon
//   pub_polygon_viz_->publish(msg_poly);
// }

} // namespace decompros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decompros::VizPoly)
