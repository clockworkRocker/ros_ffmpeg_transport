#include <ffmpeg_bridge/converters.h>
#include <ffmpeg_bridge/formats.h>

#include <ffmpeg_bridge/decoding_forwarder.hpp>

using std::placeholders::_1;
using namespace avcpp;

namespace ffmpeg_bridge {
DecodingForwarder::DecodingForwarder(const std::string& inputTopic,
                                     const std::string& outputTopic,
                                     AVCodecID codec)
    : Node("ffmpeg_decoding_forwarder"), m_decoder(codec) {
  m_sub = create_subscription<InMessage>(
      inputTopic, 10,
      // * Bind the subscription callback using a lambda
      [this](const InMessage& msg) { return subscriptionCallback(msg); });
  m_pub = create_publisher<OutMessage>(outputTopic, 10);
}

DecodingForwarder::DecodingForwarder(const std::string& inputTopic,
                                     const std::string& outputTopic,
                                     const std::string& decoderName)
    : Node("ffmpeg_decoding_forwarder"),
      m_sub(create_subscription<InMessage>(
          inputTopic, 10,
          // * Bind the subscription callback using a lambda
          [this](const InMessage& msg) { return subscriptionCallback(msg); })),
      m_pub(create_publisher<OutMessage>(outputTopic, 10)),
      m_decoder(decoderName) {}

DecodingForwarder::DecodingForwarder(const YAML::Node& config)
    : Node("ffmpeg_decoding_forwarder"),
      /* ------------------- Subscription initialization -------------------- */
      m_sub(create_subscription<InMessage>(
          config["input_topic"]  // * Check the YAML config for input topic name
              ? config["input_topic"].as<std::string>()
              : "/packet",
          10,
          // * Bind the subscription callback using a lambda
          [this](const InMessage& msg) { return subscriptionCallback(msg); })),

      /* --------------------- Publisher initialization --------------------- */
      m_pub(create_publisher<OutMessage>(
          config["output_topic"]  // * Check the YAML config for an output topic
                                  // name
              ? config["output_topic"].as<std::string>()
              : "/image_decoded",
          10)),

      /* ------------------ Internal decoder initialization ----------------- */
      m_decoder(
          config["decoder"]["name"]  // * Check the YAML config for decoder name
              ? config["decoder"]["name"].as<std::string>().c_str()
              : "hevc_cuvid"),

      /* ------------------- Decoder options initialization ------------------
       */
      m_options(config["decoder"]["options"]) {
  RCLCPP_INFO(get_logger(),
              "Created decoder. \nSubscribed to %s. Publishing packets on %s",
              m_sub->get_topic_name(), m_pub->get_topic_name());
}

int DecodingForwarder::fillPacket(const InMessage& msg) {
  int ret = 0;

  // * Realloc data if necessary
  if (static_cast<unsigned>(m_packet.size()) != msg.size) {
    av_packet_unref(m_packet.raw());
    ret = av_new_packet(m_packet.raw(), msg.size);
    if (ret < 0 || static_cast<unsigned>(m_packet.size()) != msg.size) {
      RCLCPP_ERROR(get_logger(), "Failed to allocate packet");
      return ret;
    }
  }

  // * Fill the fields
  if (msg.keyframe) m_packet.raw()->flags |= AV_PKT_FLAG_KEY;
  memcpy(m_packet.raw()->data, &msg.data[0], msg.size);
  m_packet.setCodecID(m_decoder.id());
  m_packet.raw()->pts = msg.pts;

  return ret;
}

int DecodingForwarder::reconfigureDecoder(const InMessage& msg) {
  int ret = 0;

  m_decoder.reset();

  if (m_options)
    for (auto option : m_options) {
      ret = m_decoder.setOption(option.first.as<std::string>(),
                                option.second.as<std::string>());
      if (ret < 0)
        RCLCPP_WARN(get_logger(), "Option not set: %s",
                    option.first.as<std::string>().c_str());
    }

  m_decoder.setFrameWidth(msg.width);
  m_decoder.setFrameHeight(msg.height);
  m_decoder.setFramePixelFormat(static_cast<AVPixelFormat>(msg.coded_pix_fmt));
  m_decoder.setBitrate(m_decoder.ACCORDINGLY);

  return m_decoder.open();
}

void DecodingForwarder::subscriptionCallback(const InMessage& msg) {
  int ret = 0;
  static int recievedFrames = 0;
  static double encoderTime = 0;
  static double messageTime = 0.;
  static double decodingTime = 0;
  static double conversionTime = 0.;
  static double publishingTime = 0;
  static double totalLatency = 0.;

  fillPacket(msg);
  encoderTime +=
      (rclcpp::Time(msg.header.stamp) - UNPACK_STAMP(msg.pts)).seconds() * 1000;
  messageTime += (get_clock()->now() - msg.header.stamp).seconds() * 1000;

  if (m_packet.hasKeyframe()) m_recievedKeyframe = true;

  // * If not a single keyframe has been recieved, the decoder has nothing to
  //   start decoding from
  if (!m_recievedKeyframe) return;

  if (msg.width != m_decoder.frameWidth() ||
      msg.height != m_decoder.frameHeight() ||
      static_cast<AVPixelFormat>(msg.coded_pix_fmt) != m_decoder.pixelFormat()) {
    ret = reconfigureDecoder(msg);
    if (ret < 0)
      throw std::runtime_error(
          "Could not reconfigure decoder after new message");
  }

  ret = m_decoder.decode(m_packet);
  if (ret < 0) throw std::runtime_error(avcpp::getErrorMessage(ret));
  if (ret == 1) return;

  auto tic = get_clock()->now();

  OutMessage outMsg;
  outMsg.set__width(m_decoder.frameWidth());
  outMsg.set__height(m_decoder.frameHeight());
  outMsg.set__encoding(getROSEncoding(msg.pix_fmt));
  outMsg.set__is_bigendian(AV_HAVE_BIGENDIAN);

  outMsg << m_decoder.getFrame();

  outMsg.header.set__stamp(get_clock()->now());
  outMsg.header.set__frame_id(msg.header.frame_id);

  auto toc = get_clock()->now();

  m_pub->publish(outMsg);

  conversionTime += (toc - tic).seconds() * 1000;
  publishingTime += (get_clock()->now() - toc).seconds() * 1000;

  ++recievedFrames;

  // * Calculate latency
  totalLatency +=
      (get_clock()->now() - UNPACK_STAMP(m_decoder.getFrame().pts()))
          .seconds() *
      1000;

  if (recievedFrames == 256) {
    decodingTime = totalLatency - (encoderTime + messageTime + conversionTime +
                                   publishingTime);
    RCLCPP_INFO(get_logger(),
                "\nAvg. encoder delay: %lf ms"
                "\nAvg. packet time: %lf ms"
                "\nAvg. decoding time: %lf ms"
                "\nAvg. conversion time: %lf ms"
                "\nAvg. publish time: %lf ms"
                "\nTotal latency: %lf ms",
                encoderTime / recievedFrames, messageTime / recievedFrames,
                decodingTime / recievedFrames, conversionTime / recievedFrames,
                publishingTime / recievedFrames, totalLatency / recievedFrames);
    encoderTime = 0.;
    messageTime = 0.;
    conversionTime = 0.;
    publishingTime = 0.;
    totalLatency = 0.;
    recievedFrames = 0;
  }
}

}  // namespace ffmpeg_bridge