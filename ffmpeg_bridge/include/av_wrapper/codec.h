#ifndef AV_WRAPPER_CODEC_H
#define AV_WRAPPER_CODEC_H

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
}

#include <string>

namespace avwrapper {
/**
 * A base class for encoders and decoders. Wraps around a AVCodecContext struct
 * pointer
 */
class CodecBase {
 public:  // * Types
  enum {
    /// @brief Pass this to setBitrate to automatically calculate the size for
    ///        given frame parameters
    ACCORDINGLY = -2
  };

 public:  // * Constructors
  /// Destructor
  ~CodecBase();

 public:  // * Parameter getters
  /// Get codec bitrate
  inline int bitrate() const { return m_ctx ? m_ctx->bit_rate : -1; }

  /// Get the first pixel format that is supported by the codec
  inline AVPixelFormat defaultPixFmt() const {
    // if (std::string(name()).find_first_of("nvenc") != std::string::npos)
    //   return AV_PIX_FMT_NV12;
    return m_ctx->codec->pix_fmts[0];
  }

  /// Get codec frame rate
  inline AVRational framerate() const {
    return m_ctx ? m_ctx->framerate : AVRational{0, 1};
  }

  /// Get frame width
  inline int frameWidth() const { return m_ctx ? m_ctx->width : -1; }

  /// Get frame height
  inline int frameHeight() const { return m_ctx ? m_ctx->height : -1; }

  /// Get GOP size
  inline int gopSize() const { return m_ctx ? m_ctx->gop_size : -1; }

  /// Get codec ID
  inline AVCodecID id() const {
    return m_ctx ? m_ctx->codec_id : AV_CODEC_ID_NONE;
  }

  /// Get codec name
  inline const char* name() const {
    return m_ctx ? m_ctx->codec->name : "";
  }

  /// Get frame pixel format
  inline AVPixelFormat pixelFormat() const {
    return m_ctx ? m_ctx->pix_fmt : AV_PIX_FMT_NONE;
  }

  /// Get time step for the codec
  inline AVRational timeBase() const {
    return m_ctx ? m_ctx->time_base : AVRational{0, 1};
  }

 public:  // * Parameter setters
  inline void setBitrate(int bitrate) {
    if (m_ctx) {
      m_ctx->bit_rate =
          (bitrate == ACCORDINGLY)
              ? av_image_get_buffer_size(m_ctx->pix_fmt, m_ctx->width,
                                         m_ctx->height, 32)
              : bitrate;
    };
  }

  inline void setFrameWidth(int width) {
    if (m_ctx) m_ctx->width = width;
  }

  inline void setFrameHeight(int height) {
    if (m_ctx) m_ctx->height = height;
  }

  inline void setFramerate(AVRational framerate) {
    if (m_ctx) m_ctx->framerate = framerate;
  }

  inline void setFramePixelFormat(AVPixelFormat format) {
    if (m_ctx) m_ctx->pix_fmt = format;
  }

  inline void setGOPSize(int size) {
    if (m_ctx) m_ctx->gop_size = size;
  }

  inline void setTimeBase(AVRational timeBase) {
    if (m_ctx) m_ctx->time_base = timeBase;
  }

 public:  // * Encoder internal option setters
  inline int setOption(const std::string& option, const std::string& value) {
    if (m_ctx)
      return av_opt_set(m_ctx->priv_data, option.c_str(), value.c_str(), 0);
    return 1;
  }

  inline int setOption(const std::string& option, int value) {
    if (m_ctx)
      return av_opt_set_int(m_ctx->priv_data, option.c_str(), value, 0);
    return 1;
  }

  inline int setOption(const std::string& option, double value) {
    if (m_ctx)
      return av_opt_set_double(m_ctx->priv_data, option.c_str(), value, 0);

    return 1;
  }

 public:  // * Pipeline functions

  /**
   * @brief Initialize the codec for encoding/decoding
   *
   * @details It is important to set all the necessary parameters using
   * corresponding setters BEFORE calling this method because it is impossible
   * to make changes to those parameters after initialization.
   *
   * @return 0 on success or a negative error code on fail
   */
  int open();

  /**
   * @brief Close and reset the codec.
   *
   * @warning Doing this will make the codec unusable until the next open() call
   */
  void reset();

 protected:
  inline CodecBase(const AVCodec* codec)
      : /* m_codec(codec), */ m_ctx(avcodec_alloc_context3(codec)) {}

 protected:
  // AVCodec* m_codec = nullptr;
  AVCodecContext* m_ctx = nullptr;
};

}  // namespace avwrapper
#endif  // AV_WRAPPER_CODEC_H