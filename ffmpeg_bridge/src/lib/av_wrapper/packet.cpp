#include <av_wrapper/packet.h>

/* ============================== WRAP FUNCTION ============================= */

namespace avwrapper {

Packet cpp__wrap(AVPacket* packet) {
  Packet wrap;
  av_packet_free(&wrap.m_packet);
  wrap.m_packet = packet;

  return wrap;
}

}
/* ====================== CONSTRUCTORS AND ASSIGNMENTS ====================== */

namespace avwrapper {

Packet::Packet() : m_packet(av_packet_alloc()) {}

Packet::Packet(const AVPacket* ptr)
    : m_packet(av_packet_clone(ptr)) {}

Packet::Packet(const Packet& other)
    : m_packet(av_packet_clone(other.m_packet)), m_codec_id(other.m_codec_id) {}

Packet::Packet(Packet&& other)
    : m_packet(av_packet_alloc()), m_codec_id(other.m_codec_id) {
  av_packet_move_ref(m_packet, other.m_packet);
}

Packet& Packet::operator=(const Packet& other) {
  av_packet_unref(m_packet);
  av_packet_ref(m_packet, other.m_packet);

  m_codec_id = other.m_codec_id;

  return *this;
}

Packet& Packet::operator=(Packet&& other) {
  av_packet_unref(m_packet);
  av_packet_move_ref(m_packet, other.m_packet);

  m_codec_id = other.m_codec_id;

  return *this;
}

Packet::~Packet() { av_packet_free(&m_packet); }

}