
#ifndef TF2_DECOMPROS_MSGS__TF2_GEOMETRY_MSGS_HPP_
#define TF2_DECOMPROS_MSGS__TF2_GEOMETRY_MSGS_HPP_

#include <array>
#include <string>

//#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#ifdef TF2_CPP_HEADERS
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "tf2/convert.h"
#include "tf2_ros/buffer_interface.h"

namespace tf2 {

using namespace decomp_ros_msgs::msg;

/****************/
/** Polyhedron **/
/****************/

/** \brief Apply a geometry_msgs TransformStamped to an decomp_ros Polyhedron
 * type. This function is a specialization of the doTransform template defined
 * in tf2/convert.h. \param p_in The polyhedron to transform \param p_out The
 * transformed polyhedron \param transform The timestamped transform to apply,
 * as a TransformStamped message.
 */
template <>
inline void doTransform(const Polyhedron &p_in, Polyhedron &p_out,
                        const geometry_msgs::msg::TransformStamped &transform) {

  for (geometry_msgs::msg::Point _p : p_in.ps) {
    geometry_msgs::msg::Point __p;
    doTransform(_p, __p, transform);
    p_out.ps.push_back(__p);
  }
  for (geometry_msgs::msg::Vector3 _v : p_in.ns) {
    geometry_msgs::msg::Vector3 __v;
    doTransform(_v, __v, transform);
    p_out.ns.push_back(__v);
  }
}

/********************/
/** PolyhedronStamped **/
/********************/

/** \brief Extract a timestamp from the header of a Polyhedron message.
 * This function is a specialization of the getTimestamp template defined in
 * tf2/convert.h. \param p PolyhedronStamped message to extract the timestamp
 * from. \return The timestamp of the message.
 */
template <> inline tf2::TimePoint getTimestamp(const PolyhedronStamped &p) {
  return tf2_ros::fromMsg(p.header.stamp);
}

/** \brief Extract a frame ID from the header of a PolyhedronStamped message.
 * This function is a specialization of the getFrameId template defined in
 * tf2/convert.h. \param p PolyhedronStamped message to extract the frame ID
 * from. \return A string containing the frame ID of the message.
 */
template <> inline std::string getFrameId(const PolyhedronStamped &p) {
  return p.header.frame_id;
}

/** \brief Apply a geometry_msgs TransformStamped to an decomp_ros_msgs
 * PolyhedronStamped type. This function is a specialization of the doTransform
 * template defined in tf2/convert.h. \param t_in The vector to transform, as a
 * timestamped PolyhedronStamped message. \param t_out The transformed vector,
 * as a timestamped PolyhedronStamped  message. \param transform The timestamped
 * transform to apply, as a TransformStamped message.
 */
template <>
inline void doTransform(const PolyhedronStamped &p_in, PolyhedronStamped &p_out,
                        const geometry_msgs::msg::TransformStamped &transform) {
  doTransform(p_in.poly, p_out.poly, transform);
  p_out.header.stamp = transform.header.stamp;
  p_out.header.frame_id = transform.header.frame_id;
}

} // namespace tf2

#endif // TF2_DECOMPROS_MSGS__TF2_GEOMETRY_MSGS_HPP_
