#ifndef DECOMPROS_VIZPOLY
#define DECOMPROS_VIZPOLY

#include "precompile.hpp"
#include <decomp_geometry/geometric_utils.h>
#include <decomp_util/line_segment.h>
#include <decomp_util/seed_decomp.h>
#include <iostream>
#include <memory>

namespace decompros {

class VizPoly : public rclcpp::Node {

public:
  explicit VizPoly(const rclcpp::NodeOptions &options);

protected:
  // subs
  rclcpp::Subscription<decomp_ros_msgs::msg::PolyhedronStamped>::SharedPtr
      sub_polygon_;
  rclcpp::Subscription<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr
      sub_polygon_array_;

  // pubs
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      pub_polygon_viz2_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      pub_polygon_array_viz2_;

  // params
  double color_r_ = 0.0f;
  double color_g_ = 1.0f;
  double color_b_ = 0.0f;
  double color_a_ = 1.0f;


  // vars

  // methods

  void
  callback(const decomp_ros_msgs::msg::PolyhedronStamped::SharedPtr msg) const;
  void array_callback(
      const decomp_ros_msgs::msg::PolyhedronArray::SharedPtr array_msg) const;

  void add_to_marker_msg(visualization_msgs::msg::Marker &marker_msg,
                         Polyhedron3D &poly) const;

  Polyhedron3D convertToPolyhedron(decomp_ros_msgs::msg::Polyhedron msg) const;
};

} // namespace decompros

#endif // DECOMPROS_VIZPOLY
