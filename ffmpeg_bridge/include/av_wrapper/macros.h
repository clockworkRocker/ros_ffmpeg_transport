#ifndef AV_WRAPPER_MACROS_H
#define AV_WRAPPER_MACROS_H

extern "C"{
#include <libavutil/error.h>
}

#include <iostream>

#define AV_WRAPPER_ERRORS_LIST                                                \
  AV_WRAPPER_ERROR_COUPLE(-20, CodecNotOpen,                                  \
                          "Codec is not open for encoding/decoding.")         \
  AV_WRAPPER_ERROR_COUPLE(-21, InvalidPixelFormat,                            \
                          "Codec cannot process the given pixel format")      \
  AV_WRAPPER_ERROR_COUPLE(-22, InvalidDimensions,                             \
                          "Codec cannot process frame with given dimensions") \
  AV_WRAPPER_ERROR_COUPLE(-23, WrongDecoder,                                  \
                          "Packet was encoded with different encoder")        \
  AV_WRAPPER_ERROR_COUPLE(-24, SendFrameFailed,                               \
                          "Error sending frame to encoder")                   \
  AV_WRAPPER_ERROR_COUPLE(-25, SendPacketFailed,                              \
                          "Error sending packet to decoder")                  \
  AV_WRAPPER_ERROR_COUPLE(-26, ReceiveFrameFailed,                            \
                          "Error receiving frame from decoder")               \
  AV_WRAPPER_ERROR_COUPLE(-27, ReceivePacketFailed,                           \
                          "Error receiving packet from encoder")
/* ========================================================================== */

#define AV_WRAPPER_ERROR_COUPLE(num, name, message) name = num,

namespace avwrapper {

/// @brief Codec errors enum
enum AVCodecWrapperError { AV_WRAPPER_ERRORS_LIST };

}  // namespace avwrapper

#undef AV_WRAPPER_ERROR_COUPLE

/* ======================= ERROR MESSAGE FUNCTION =========================== */

#define AV_WRAPPER_ERROR_COUPLE(num, name, message) \
  case name:                                        \
    return message;

namespace avwrapper {

enum { MaxErrorBufSize = 64 };
/// @brief A string buffer for error messages
static char ErrorBuf[MaxErrorBufSize];

/// Get a corresponding error message from error code
inline const char* getErrorMessage(int code) {
  switch (code) {
    AV_WRAPPER_ERRORS_LIST

    default:
      return av_make_error_string(ErrorBuf, MaxErrorBufSize, code);
  }
}
}  // namespace avwrapper

#undef AV_WRAPPER_ERROR_COUPLE

/* ========================================================================== */

/// Compact error report macro
#define AV_WRAPPER_ERROR_REPORT(code)                                   \
  do {                                                                  \
    std::cerr << __FUNCTION__ << ": " << getErrorMessage(code) << "\n"; \
    return code;                                                        \
  } while (0)

/// Check if the code is negative and return the error if that's the case
#define AV_WRAPPER_ERROR_CHECK(code) \
  if (code < 0) AV_WRAPPER_ERROR_REPORT(code);

#undef AV_WRAPPER_ERRORS_LIST

#endif  // AV_WRAPPER_MACROS_H