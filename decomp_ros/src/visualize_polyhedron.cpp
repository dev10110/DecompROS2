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

  pub_viz_=
      this->create_publisher<visualization_msgs::msg::MarkerArray>("sfc/viz", 1);

  pub_array_viz_ =
     this->create_publisher<visualization_msgs::msg::MarkerArray>("sfc_array/viz",
                                                             1);

  color_r_ = this->declare_parameter<double>("color_r", color_r_);
  color_g_ = this->declare_parameter<double>("color_g", color_g_);
  color_b_ = this->declare_parameter<double>("color_b", color_b_);
  color_a_ = this->declare_parameter<double>("color_a", color_a_);
  line_w_ = this->declare_parameter<double>("edge_width", line_w_);

  // create the subscribers
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos_sensor_data = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  sub_ =
      this->create_subscription<decomp_ros_msgs::msg::PolyhedronStamped>(
          "sfc", qos_sensor_data, std::bind(&VizPoly::callback, this, _1));

  sub_array_ =
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

  visualization_msgs::msg::MarkerArray marker_msg;

  bool suc = convertToViz(marker_msg, msg->header, poly);
  if (!suc)
    return;

  // publish the message
  pub_viz_->publish(marker_msg);

}

bool VizPoly::convertToViz(visualization_msgs::msg::MarkerArray & marker_msg, std_msgs::msg::Header header, Polyhedron3D & poly) const
{

  // first extract the faces of the polyhedron
  vec_E<vec_Vec3f> faces = cal_vertices(poly);
  
  if (faces.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Polyhedron is empty! (has 0 faces)");
    return false;
  }

  // construct faces message
  visualization_msgs::msg::Marker faces_marker;
  visualization_msgs::msg::Marker edges_marker;

  bool suc = createFacesMarker(faces_marker, header, faces);
  if (!suc)
    return false;

  suc = createEdgesMarker(edges_marker, header, faces);
  if (!suc)
    return false;
  edges_marker.header = header;

  // combine the two msgs
  marker_msg.markers.push_back(faces_marker);
  marker_msg.markers.push_back(edges_marker);

  RCLCPP_WARN(get_logger(), "added both messages");

  return true;
}

bool VizPoly::createFacesMarker(visualization_msgs::msg::Marker & marker, std_msgs::msg::Header header, vec_E<vec_Vec3f> & faces) const 
{
  marker.header = header;
  marker.ns = "faces";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.color.r = color_r_;
  marker.color.g = color_g_;
  marker.color.b = color_b_;
  marker.color.a = color_a_;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

  bool face_added = false;
  
  for (auto verts : faces) {
    // each face has a list of points making up the face
    size_t N = verts.size();
    if (N < 3) {
      RCLCPP_WARN(this->get_logger(),
                  "Polyhedron face is empty! (has < 3 verts)");
    }
    for (size_t i = 1; i < N - 1; i++) {
      marker.points.push_back(toPoint(verts[0])); // form the triangle
      marker.points.push_back(toPoint(verts[i]));
      marker.points.push_back(toPoint(verts[i + 1]));
      face_added = true;
    }
  }

  return face_added;

}

bool VizPoly::createEdgesMarker(visualization_msgs::msg::Marker & marker, std_msgs::msg::Header header, vec_E<vec_Vec3f> & faces) const
{

  marker.header = header;
  marker.ns = "edges";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.color.r = color_r_;
  marker.color.g = color_g_;
  marker.color.b = color_b_;
  marker.color.a = 1.0; // force the edges to have alpha=1.0;
  marker.scale.x = line_w_; 

  bool edge_added = false;
  
  for (auto verts : faces) {
    // each face has a list of points making up the face
    size_t N = verts.size();
    if (N < 2) {
      RCLCPP_WARN(this->get_logger(),
                  "Polyhedron face is empty! (has < 2 verts)");
    }
    for (size_t i = 0; i < N; i++) {
      marker.points.push_back(toPoint(verts[i % N]));
      marker.points.push_back(toPoint(verts[(i + 1) % N ]));
      edge_added = true;
    }
  }

  return edge_added;

}

void VizPoly::array_callback(
    const decomp_ros_msgs::msg::PolyhedronArray::SharedPtr array_msg) const {

  size_t N_polys = array_msg->polys.size();
  if (N_polys == 0) {
    RCLCPP_WARN(this->get_logger(), "skipping sfc array has no polyhedrons");
    return;
  }
  

  visualization_msgs::msg::MarkerArray marker_msg;
  
  for (auto poly_msg : array_msg->polys)
  {
    Polyhedron3D poly = convertToPolyhedron(poly_msg.poly);
    bool suc = convertToViz(marker_msg, array_msg->header, poly);
    if (!suc)
      return; // failed to convert one of the polyhedrons
  }

  // now publish the full msg
  pub_array_viz_->publish(marker_msg);
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

} // namespace decompros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decompros::VizPoly)
