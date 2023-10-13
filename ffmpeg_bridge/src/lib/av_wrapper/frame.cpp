#include "av_wrapper/frame.h"

extern "C" {
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <exception>

namespace avwrapper {

Frame cpp__wrap(AVFrame* frame) {
  Frame wrap;
  av_frame_free(&wrap.m_frame);

  wrap.m_frame = frame;

  return wrap;
}

Frame::Frame(int width, int height, AVPixelFormat format)
    : m_frame(av_frame_alloc()) {
  m_frame->width = width;
  m_frame->height = height;
  m_frame->format = format;

  if (width > 0 && height > 0 && format != AV_PIX_FMT_NONE) {
    int ret = av_frame_get_buffer(m_frame, 32);
    if (ret < 0) throw std::bad_alloc();
  }
}

Frame::Frame(const AVFrame* ptr) : m_frame(av_frame_clone(ptr)){};

Frame::Frame(const Frame& other) : m_frame(av_frame_clone(other.m_frame)) {}

Frame::Frame(Frame&& other) : m_frame(av_frame_alloc()) {
  av_frame_unref(m_frame);
  av_frame_move_ref(m_frame, other.m_frame);
}

Frame& Frame::operator=(const Frame& other) {
  av_frame_unref(m_frame);
  av_frame_ref(m_frame, other.m_frame);

  return *this;
}

Frame& Frame::operator=(Frame&& other) {
  std::swap(m_frame, other.m_frame);

  return *this;
}

Frame::~Frame() { av_frame_free(&m_frame); }

/* ========================================================================== */

size_t Frame::datasize() const {
  int result =
      av_image_get_buffer_size(static_cast<AVPixelFormat>(m_frame->format),
                               m_frame->width, m_frame->height, 1);
  return result > 0 ? static_cast<size_t>(result) : 0;
}

/* ========================================================================== */

Frame& Frame::toFormat(AVPixelFormat format) {
  if (format == static_cast<AVPixelFormat>(m_frame->format))
    return *this;

  Frame another(width(), height(), format);
  SwsContext* conversion = sws_getContext(
      m_frame->width, m_frame->height,
      static_cast<AVPixelFormat>(m_frame->format), m_frame->width,
      m_frame->height, format, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
  sws_scale(conversion, m_frame->data, m_frame->linesize, 0, m_frame->height,
            another.raw()->data, another.raw()->linesize);
  sws_freeContext(conversion);

  *this = std::move(another);

  return *this;
}
}  // namespace avwrapper