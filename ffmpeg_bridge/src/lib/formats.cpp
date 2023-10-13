#include "ffmpeg_bridge/formats.h"

#define KRYA(A, B) \
  if (rosEncoding == A) return B;

AVPixelFormat getPixelFormat(const std::string& rosEncoding) {
  CONVERTER_FORMAT_PAIRS

  // * If all else fails, we don't know how to handle the encoding
  return AV_PIX_FMT_NONE;
}

#undef KRYA
#define KRYA(A, B) \
  case B:          \
    return A;

std::string getROSEncoding(int pixelFormat) {
  switch (pixelFormat) {
    CONVERTER_FORMAT_PAIRS
    default:
      return "";
  }
}

#undef KRYA
