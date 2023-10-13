#ifndef AV_WRAPPER_PACKET_H
#define AV_WRAPPER_PACKET_H

extern "C" {
#include <libavcodec/codec.h>
#include <libavcodec/packet.h>
}

/* ========================== FORWARD DECLARATIONS ========================== */

namespace avwrapper {

class Packet;

}
/* ============================== WRAP FUNCTION ============================= */

namespace avwrapper {
/**
 * @return A wrapper object for the given pointer. No additional allocation
 * happens
 */
Packet cpp__wrap(AVPacket* packet);
}  // namespace avwrapper
/* ============================ PUBLIC INTERFACE ============================ */

namespace avwrapper {
  /**
 * A C++ wrapper around a libav AVPacket struct pointer
 */
class Packet {
 public:  // * Types and friends
  friend Packet cpp__wrap(AVPacket* packet);

 public:  // * Constructors and assignments
  /// Basic constructor
  Packet();

  /// Copy constructor
  Packet(const Packet& other);

  /// Make a copy from a raw pointer (Yes, it does copy everything. To avoid
  /// that consider using cpp__wrap() function instead)
  Packet(const AVPacket* ptr);

  /// Move constructor
  Packet(Packet&& other);

  /// Assign operator
  Packet& operator=(const Packet& other);

  /// Move-assign operator
  Packet& operator=(Packet&& other);

  /// Destructor
  ~Packet();

 public:  // * Accessors
  /// @return The presentation timestamp
  inline int64_t pts() const { return m_packet->pts; }

  /// @return The ID of the codec used to create the packet
  inline AVCodecID codecID() const { return m_codec_id; }

  /// @return The raw packet pointer if necessary
  inline AVPacket* raw() { return m_packet; }

  /// @return The pointer to const packet
  inline const AVPacket* raw() const { return m_packet; }

  /// @return True if the packet contains a keyframe
  inline bool hasKeyframe() const { return m_packet->flags & AV_PKT_FLAG_KEY; }

  /// @return Size of packet buffer in bytes
  inline int size() const { return m_packet->size; }

  /// @return A pointer to the packet data buffer
  inline uint8_t* buffer() { return m_packet->data; }

  /// @return A pointer to const packet data buffer
  inline const uint8_t* buffer() const { return m_packet->data; }

 public:  // * Setters
  inline void setCodecID(AVCodecID id) { m_codec_id = id; }

 private:
  AVPacket* m_packet = nullptr;
  AVCodecID m_codec_id = AV_CODEC_ID_NONE;
};

}
#endif  // AV_WRAPPER_PACKET_H