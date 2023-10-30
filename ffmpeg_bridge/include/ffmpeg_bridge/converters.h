#ifndef FFMPEG_BRIDGE__CONVERTERS_H
#define FFMPEG_BRIDGE__CONVERTERS_H

#include <avcpp/AV.hpp>
#include <ffmpeg_interfaces/msg/av_packet.hpp>
#include <sensor_msgs/msg/image.hpp>

/// Pack ROS Timestamp into an avcpp pts
#define PACK_STAMP(stamp) \
  (static_cast<int64_t>(stamp.sec) << 32 | static_cast<int64_t>(stamp.nanosec))

/// Unpack ROS Timestamp from avcpp pts
#define UNPACK_STAMP(pts)                        \
  rclcpp::Time(static_cast<uint32_t>(pts >> 32), \
               static_cast<int32_t>((pts << 32) >> 32), RCL_ROS_TIME)

namespace ffmpeg_bridge {
namespace converters {
/** @brief Set the interpolation method used in sws_scale function during
 *         conversion between AVFrames and ROS2 Images
 * @param method Should be one of the SWS defined methods
 * @return True if the method was successfully set
 */
bool setConversionMethod(unsigned method);
}  // namespace converters
}  // namespace ffmpeg_bridge

/**
 * @brief Fill up the frame data from sensor_msgs/msg/Image. If the dimensions
 *        or formats do not match, the image will be scaled accordingly using
 *        sws_scale
 */
const sensor_msgs::msg::Image& operator>>(const sensor_msgs::msg::Image& msg,
                                          avcpp::Frame& frame);

/**
 * @brief Fill up the message data from the frame. If the dimensions or formats
 * do not match, the image will be scaled accordingly using sws_scale.
 * @warning This only fills up the data buffer. Width/height and the encoding
 * should be set beforehand
 */
sensor_msgs::msg::Image& operator<<(sensor_msgs::msg::Image& msg,
                                    const avcpp::Frame& frame);

#endif  // FFMPEG_BRIDGE__CONVERTERS_H