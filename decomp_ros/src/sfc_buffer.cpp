// collects the past N sfcs together and publishes them
// uses a circular buffer internally
// also converts them to specified frame

#include "decomp_ros/sfc_buffer.hpp"

namespace decompros {

SFCBuffer::SFCBuffer(const rclcpp::NodeOptions &options)
    : Node("sfc_buffer", options), buffer_(3) {

  RCLCPP_INFO(this->get_logger(), "starting...");

  using std::placeholders::_1;

  // create parameters
  target_frame_ = "vicon/world";
  home_sfc_xmin_ = -0.5; // m
  home_sfc_xmax_ = 0.5;
  home_sfc_ymin_ = -0.5;
  home_sfc_ymax_ = 0.5;
  home_sfc_zmin_ = -0.5;
  home_sfc_zmax_ = 1.5;

  // construct the home_sfc
  {
    // construct home sfc
    home_sfc_.header.frame_id = target_frame_;
    { // xmin
      geometry_msgs::msg::Point p;
      geometry_msgs::msg::Vector3 n;
      p.x = home_sfc_xmin_;
      n.x = -1;
      home_sfc_.poly.ps.push_back(p);
      home_sfc_.poly.ns.push_back(n);
    }
    { // xmax
      geometry_msgs::msg::Point p;
      geometry_msgs::msg::Vector3 n;
      p.x = home_sfc_xmax_;
      n.x = 1;
      home_sfc_.poly.ps.push_back(p);
      home_sfc_.poly.ns.push_back(n);
    }
    { // ymin
      geometry_msgs::msg::Point p;
      geometry_msgs::msg::Vector3 n;
      p.y = home_sfc_ymin_;
      n.y = -1;
      home_sfc_.poly.ps.push_back(p);
      home_sfc_.poly.ns.push_back(n);
    }
    { // ymax
      geometry_msgs::msg::Point p;
      geometry_msgs::msg::Vector3 n;
      p.y = home_sfc_ymax_;
      n.y = 1;
      home_sfc_.poly.ps.push_back(p);
      home_sfc_.poly.ns.push_back(n);
    }
    { // zmin
      geometry_msgs::msg::Point p;
      geometry_msgs::msg::Vector3 n;
      p.z = home_sfc_zmin_;
      n.z = -1;
      home_sfc_.poly.ps.push_back(p);
      home_sfc_.poly.ns.push_back(n);
    }
    { // zmax
      geometry_msgs::msg::Point p;
      geometry_msgs::msg::Vector3 n;
      p.z = home_sfc_zmax_;
      n.z = 1;
      home_sfc_.poly.ps.push_back(p);
      home_sfc_.poly.ns.push_back(n);
    }
  }

  // create publishers
  pub_poly_array_ =
      this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>("sfc_array",
                                                                    10);

  // create the tf buffer
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // create the poly subscriber
  std::chrono::duration<int> buffer_timeout(1);
  sub_poly_.subscribe(this, "sfc");
  tf2_filter_ = std::make_shared<
      tf2_ros::MessageFilter<decomp_ros_msgs::msg::PolyhedronStamped>>(
      sub_poly_, *tf2_buffer_, target_frame_, 100,
      this->get_node_logging_interface(), this->get_node_clock_interface(),
      buffer_timeout);
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&SFCBuffer::poly_callback, this);

  RCLCPP_INFO(this->get_logger(), "Starting sfc_buffer");
}

void SFCBuffer::poly_callback(
    const decomp_ros_msgs::msg::PolyhedronStamped::SharedPtr msg) {

  // convert to world frame
  decomp_ros_msgs::msg::PolyhedronStamped new_poly;

  try {
    tf2_buffer_->transform(*msg, new_poly, target_frame_);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what());
  }

  // add to circular buffer
  buffer_.push_back(new_poly);

  // publish array
  decomp_ros_msgs::msg::PolyhedronArray poly_array_msg;
  poly_array_msg.header.frame_id = target_frame_;
  poly_array_msg.header.stamp = this->get_clock()->now();
  for (auto m : buffer_) {
    poly_array_msg.polys.push_back(m);
  }

  home_sfc_.header.stamp = poly_array_msg.header.stamp;
  poly_array_msg.polys.push_back(home_sfc_);

  pub_poly_array_->publish(poly_array_msg);

  RCLCPP_DEBUG(this->get_logger(), "published array");
}

} // namespace decompros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decompros::SFCBuffer)
