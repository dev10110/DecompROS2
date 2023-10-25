#ifndef DECOMPROS_VIZPOLY
#define DECOMPROS_VIZPOLY

#include "precompile.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <decomp_geometry/geometric_utils.h>
#include <decomp_util/line_segment.h>
#include <decomp_util/seed_decomp.h>
#include <iostream>
#include <memory>

namespace decompros {


  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Header = std_msgs::msg::Header;
  using Polyhedron = decomp_ros_msgs::msg::Polyhedron;
  using PolyhedronStamped = decomp_ros_msgs::msg::PolyhedronStamped;
  using PolyhedronArray = decomp_ros_msgs::msg::PolyhedronArray;
  using Faces = vec_E<vec_Vec3f>;

class VizPoly : public rclcpp::Node {

public:
  explicit VizPoly(const rclcpp::NodeOptions &options);

protected:
  // subs
  rclcpp::Subscription<PolyhedronStamped>::SharedPtr
      sub_;
  rclcpp::Subscription<PolyhedronArray>::SharedPtr
      sub_array_;

  // pubs
  rclcpp::Publisher<MarkerArray>::SharedPtr
      pub_viz_;
  rclcpp::Publisher<MarkerArray>::SharedPtr
      pub_array_viz_;

  // params
  double color_r_ = 0.0;
  double color_g_ = 1.0;
  double color_b_ = 0.0;
  double color_a_ = 1.0;
  double line_w_ = 0.01;


  // vars

  // methods

  void callback(const PolyhedronStamped::SharedPtr msg) const;
  void array_callback(const PolyhedronArray::SharedPtr array_msg) const;
  
  Polyhedron3D convertToPolyhedron(Polyhedron msg) const;

  bool convertToViz(
      MarkerArray & marker_msg, Header header, Polyhedron3D & poly) const;

  bool createFacesMarker(Marker & marker, Header header, Faces & faces) const;
  bool createEdgesMarker(Marker & marker, Header header, Faces & faces) const;

};

} // namespace decompros

#endif // DECOMPROS_VIZPOLY
