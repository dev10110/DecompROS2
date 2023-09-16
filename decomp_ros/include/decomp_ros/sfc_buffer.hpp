#ifndef decompros_sfcbuffer
#define decompros_sfcbuffer

#include "precompile.hpp"
#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include <boost/circular_buffer.hpp>
#include <chrono>
#include <iostream>
#include <memory>

namespace decompros {

class SFCBuffer : public rclcpp::Node {

public:
  explicit SFCBuffer(const rclcpp::NodeOptions &options);

protected:
  // params
  double home_sfc_xmin_, home_sfc_xmax_;
  double home_sfc_ymin_, home_sfc_ymax_;
  double home_sfc_zmin_, home_sfc_zmax_;

  std::string target_frame_;

  // vars
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<
      tf2_ros::MessageFilter<decomp_ros_msgs::msg::PolyhedronStamped>>
      tf2_filter_;

  decomp_ros_msgs::msg::PolyhedronStamped home_sfc_;

  // main buffer
  boost::circular_buffer<decomp_ros_msgs::msg::PolyhedronStamped> buffer_;

  // subs
  message_filters::Subscriber<decomp_ros_msgs::msg::PolyhedronStamped>
      sub_poly_;

  // pubs
  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr
      pub_poly_array_;

  // methods
  void
  poly_callback(const decomp_ros_msgs::msg::PolyhedronStamped::SharedPtr msg);

}; // class SFCBuffer

} // namespace decompros

#endif // decompros_sfcbuffer
