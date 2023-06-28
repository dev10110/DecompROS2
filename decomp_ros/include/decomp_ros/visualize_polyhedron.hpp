#ifndef DECOMPROS_VIZPOLY
#define DECOMPROS_VIZPOLY

#include "decomp_ros_msgs/msg/polyhedron_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <decomp_util/line_segment.h>
#include <decomp_util/seed_decomp.h>

namespace decompros {

class VizPoly : public rclcpp::Node {

public:
  explicit VizPoly(const rclcpp::NodeOptions &options);

protected:
  // subs
  rclcpp::Subscription<decomp_ros_msgs::msg::PolyhedronStamped>::SharedPtr
      sub_polygon_;

  // pubs
  // rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
  //    pub_polygon_viz_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      pub_polygon_viz2_;

  // params

  // vars

  // methods

  void
  callback(const decomp_ros_msgs::msg::PolyhedronStamped::SharedPtr msg) const;
  // void publish_polygon_stamped(std_msgs::msg::Header, Polyhedron3D) const;
  void publish_polygon_as_lines(const std_msgs::msg::Header header,
                                Polyhedron3D poly) const;
};

} // namespace decompros

#endif // DECOMPROS_VIZPOLY
