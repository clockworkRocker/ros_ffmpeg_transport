#ifndef FFMPEG_BRIDGE__CONVERTERS_H
#define FFMPEG_BRIDGE__CONVERTERS_H

#include <av_wrapper/frame.h>
#include <av_wrapper/packet.h>

#include <ffmpeg_interfaces/msg/av_packet.hpp>
#include <sensor_msgs/msg/image.hpp>

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
 *        SWScale
 */
const sensor_msgs::msg::Image& operator>>(const sensor_msgs::msg::Image& msg,
                                          avwrapper::Frame& frame);

/**
 * @brief Fill up the message data from the frame. If the dimensions or formats
 * do not match, the image will be scaled accordingly using sws_scale.
 * @warning This only fills up the data buffer. Width/height and the encoding
 * should be set beforehand
 */
sensor_msgs::msg::Image& operator<<(sensor_msgs::msg::Image& msg,
                                    const avwrapper::Frame& frame);

#endif  // ROSFFMPEG_CONVERTERS_H