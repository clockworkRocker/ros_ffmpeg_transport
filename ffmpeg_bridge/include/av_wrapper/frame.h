#ifndef AV_WRAPPER_FRAME_H
#define AV_WRAPPER_FRAME_H

extern "C" {
#include <libavutil/frame.h>
}

namespace avwrapper {
class Frame {
 public:  // * Types and friends
  friend Frame cpp__wrap(AVFrame* frame);

 public:  // * Construction
  /// Basic constructor
  Frame(int width = 0, int height = 0, AVPixelFormat format = AV_PIX_FMT_NONE);
  /// Constructor from raw AVFrame*
  Frame(const AVFrame* ptr);
  /// Copy constructor
  Frame(const Frame& other);
  /// Move constructor
  Frame(Frame&& other);
  /// Assignment
  Frame& operator=(const Frame& other);
  /// Move assignment
  Frame& operator=(Frame&& other);

  ~Frame();

 public:  // * Stats
  /// Get buffer size in bytes required for this frame
  size_t datasize() const;

 public:  // * Getters
  inline AVFrame* raw() { return m_frame; }
  inline const AVFrame* raw() const { return m_frame; }

  inline int format() const { return m_frame->format; }
  inline int width() const { return m_frame->width; }
  inline int height() const { return m_frame->height; }
  inline int rowStep() const { return m_frame->linesize[0]; }

  inline int64_t pts() const { return m_frame->pts; }

  inline bool isKeyFrame() const { return m_frame->key_frame; };

  inline uint8_t* buffer() { return m_frame->data[0]; }
  inline const uint8_t* buffer() const { return m_frame->data[0]; }

 public: // * Setters
  inline void setPts(int64_t pts) { m_frame->pts = pts; }

 public:  // * Utilities
  /// Convert a frame to another pixel formats
  Frame& toFormat(AVPixelFormat format);

 private:
  AVFrame* m_frame = nullptr;
};

/**
 * @brief Create an object around an AVFrame struct pointer. No data is copied.
 */
Frame cpp__wrap(AVFrame* frame);

}  // namespace avwrapper
#endif