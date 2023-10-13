#include "ffmpeg_bridge/converters.h"

#include <iostream>

#include "ffmpeg_bridge/formats.h"

extern "C" {
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

/* ================================== TYPES ================================= */
struct SwsContextDeleter {
  void operator()(SwsContext** ptr) {
    sws_freeContext(*ptr);
    delete ptr;
  }
};
using UniqueSwsContext = std::unique_ptr<SwsContext*, SwsContextDeleter>;

/* ======================== GLOBAL CONTEXT VARIABLES ======================== */
/// Context for converting AVFrame data to Image Message data
UniqueSwsContext FrameToImageCtx(new SwsContext*(nullptr));
/// Context for converting Image Message data into AVFrame data
UniqueSwsContext ImageToFrameCtx(new SwsContext*(nullptr));

/* ======================= GLOBAL CONVERTER VARIABLES ======================= */
namespace ffmpeg_bridge {
namespace converters {
unsigned ConversionMethod = SWS_LANCZOS;
}
}  // namespace ffmpeg_bridge
/* =========================== DATA FILL OPERATORS ========================== */

const sensor_msgs::msg::Image& operator>>(const sensor_msgs::msg::Image& msg,
                                          avwrapper::Frame& frame) {
  const uint8_t* data[] = {&msg.data[0]};
  const int linesize[] = {static_cast<int>(msg.step)};

  *ImageToFrameCtx = sws_getCachedContext(
      *ImageToFrameCtx, msg.width, msg.height, getPixelFormat(msg.encoding),
      frame.width(), frame.height(), static_cast<AVPixelFormat>(frame.format()),
      ffmpeg_bridge::converters::ConversionMethod, nullptr, nullptr, nullptr);

  sws_scale(*ImageToFrameCtx, data, linesize, 0, msg.height, frame.raw()->data,
            frame.raw()->linesize);

  return msg;
}

sensor_msgs::msg::Image& operator<<(sensor_msgs::msg::Image& msg,
                                    const avwrapper::Frame& frame) {
  AVPixelFormat format = getPixelFormat(msg.encoding);
  msg.data.resize(
      av_image_get_buffer_size(format, frame.width(), frame.height(), 32));

  uint8_t* data[] = {&msg.data[0]};
  int linesize[4];

  av_image_fill_linesizes(linesize, format, frame.width());

  *FrameToImageCtx = sws_getCachedContext(
      *FrameToImageCtx, frame.width(), frame.height(),
      static_cast<AVPixelFormat>(frame.format()), msg.width, msg.height,
      getPixelFormat(msg.encoding), ffmpeg_bridge::converters::ConversionMethod,
      nullptr, nullptr, nullptr);

  sws_scale(*FrameToImageCtx, frame.raw()->data, frame.raw()->linesize, 0,
            frame.height(), data, linesize);
  msg.set__step(linesize[0]);

  return msg;
}

/* ==================== CONVERSION MANIPULATION FUNCTIONS =================== */
namespace ffmpeg_bridge {
namespace converters {
bool setConversionMethod(unsigned method) {
  switch (method) {
    case SWS_BICUBIC:
    case SWS_BICUBLIN:
    case SWS_BILINEAR:
    case SWS_FAST_BILINEAR:
    case SWS_GAUSS:
    case SWS_LANCZOS:
    case SWS_POINT:
    case SWS_SINC:
    case SWS_SPLINE:
      std::cout << "ffmpeg_bridge: Changed conversion method.\n";
      ConversionMethod = method;
      return true;
    default:
      std::cout << "ffmpeg_bridge: Unknown conversion method.\n";
      return false;
  }
}
}  // namespace converters
}  // namespace ffmpeg_bridge