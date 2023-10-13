#ifndef FFMPEG_BRIDGE__FORMATS_H
#define FFMPEG_BRIDGE__FORMATS_H
extern "C" {
#include <libavcodec/avcodec.h>
}

#include <sensor_msgs/image_encodings.hpp>

#define CONVERTER_FORMAT_PAIRS                                              \
  KRYA(sensor_msgs::image_encodings::RGB8, AV_PIX_FMT_RGB24)                \
  KRYA(sensor_msgs::image_encodings::RGBA8, AV_PIX_FMT_RGBA)                \
  KRYA(sensor_msgs::image_encodings::RGB16, AV_PIX_FMT_RGB48)               \
  KRYA(sensor_msgs::image_encodings::RGBA16, AV_PIX_FMT_RGBA64)             \
  KRYA(sensor_msgs::image_encodings::BGR8, AV_PIX_FMT_BGR24)                \
  KRYA(sensor_msgs::image_encodings::BGRA8, AV_PIX_FMT_BGRA)                \
  KRYA(sensor_msgs::image_encodings::BGR16, AV_PIX_FMT_BGR48)               \
  KRYA(sensor_msgs::image_encodings::BGRA16, AV_PIX_FMT_BGRA64)             \
  KRYA(sensor_msgs::image_encodings::MONO8, AV_PIX_FMT_GRAY8)               \
  KRYA(sensor_msgs::image_encodings::MONO16, AV_PIX_FMT_GRAY16)             \
                                                                            \
  KRYA(sensor_msgs::image_encodings::BAYER_RGGB8, AV_PIX_FMT_BAYER_RGGB8)   \
  KRYA(sensor_msgs::image_encodings::BAYER_RGGB16, AV_PIX_FMT_BAYER_RGGB16) \
  KRYA(sensor_msgs::image_encodings::BAYER_BGGR8, AV_PIX_FMT_BAYER_BGGR8)   \
  KRYA(sensor_msgs::image_encodings::BAYER_BGGR16, AV_PIX_FMT_BAYER_BGGR16) \
  KRYA(sensor_msgs::image_encodings::BAYER_GBRG8, AV_PIX_FMT_BAYER_GRBG8)   \
  KRYA(sensor_msgs::image_encodings::BAYER_GBRG16, AV_PIX_FMT_BAYER_GBRG16) \
                                                                            \
  KRYA(sensor_msgs::image_encodings::YUV422, AV_PIX_FMT_UYVY422)

/**
 * @brief Get a corrsponding AV Pixel format from a ROS Image encoding string
 */
AVPixelFormat getPixelFormat(const std::string& rosEncoding);

/**
 * @brief Get a corresponding ROS Image encoding string from AV pixel format
 */
std::string getROSEncoding(int pixelFormat);

#endif // ROSFFMPEG_FORMATS_H