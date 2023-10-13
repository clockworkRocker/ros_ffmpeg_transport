#include "av_wrapper/codec.h"

#include "av_wrapper/macros.h"

namespace avwrapper {

CodecBase::~CodecBase() {
  if (avcodec_is_open(m_ctx)) avcodec_close(m_ctx);

  avcodec_free_context(&m_ctx);
}

int CodecBase::usePreset(const std::string& preset) {
  m_ctx->bit_rate_tolerance = 0;
  m_ctx->rc_max_rate = 0;
  m_ctx->rc_buffer_size = 0;
  m_ctx->max_b_frames = 0;
  m_ctx->me_cmp = 1;
  m_ctx->me_range = 16;
  m_ctx->qmin = 10;
  m_ctx->qmax = 51;
  m_ctx->flags |= AV_CODEC_FLAG_LOOP_FILTER | AV_CODEC_FLAG_LOW_DELAY; 
  m_ctx->me_subpel_quality = 5;
  m_ctx->i_quant_factor = 0.71;
  m_ctx->qcompress = 0.6;
  m_ctx->max_qdiff = 4;

  int ret = setOption("preset", preset);
  AV_WRAPPER_ERROR_CHECK(ret);
  ret = setOption("max_delay", "0");
  AV_WRAPPER_ERROR_CHECK(ret);
  
  return 0;
}

int CodecBase::useTune(const std::string& value) {
  int ret = setOption("tune", value);
  AV_WRAPPER_ERROR_CHECK(ret);

  return 0;
}

int CodecBase::open() {
  if (/* m_codec && */ m_ctx->codec) {
    int ret = avcodec_open2(m_ctx, /* m_codec */ nullptr, nullptr);
    AV_WRAPPER_ERROR_CHECK(ret);

    return ret;
  }
  return -1;
}

void CodecBase::reset() {
  const AVCodec* old_codec = m_ctx->codec;

  avcodec_free_context(&m_ctx);
  m_ctx = avcodec_alloc_context3(old_codec);
}
}  // namespace avwrapper