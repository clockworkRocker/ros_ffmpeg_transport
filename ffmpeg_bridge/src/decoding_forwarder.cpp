#include <av_wrapper/macros.h>
#include <ffmpeg_bridge/converters.h>
#include <ffmpeg_bridge/formats.h>

#include <ffmpeg_bridge/decoding_forwarder.hpp>

using std::placeholders::_1;
using namespace avwrapper;

namespace ffmpeg_bridge {
DecodingForwarder::DecodingForwarder(const std::string& inputTopic,
                                     const std::string& outputTopic,
                                     AVCodecID codec)
    : Node("ffmpeg_decoding_forwarder"),
      m_sub(create_subscription<InMessage>(
          inputTopic, 10,
          std::bind(&DecodingForwarder::subscriptionCallback, this, _1))),
      m_pub(create_publisher<OutMessage>(outputTopic, 10)),
      m_decoder(codec) {}

DecodingForwarder::DecodingForwarder(const std::string& inputTopic,
                                     const std::string& outputTopic,
                                     const std::string& decoderName)
    : Node("ffmpeg_decoding_forwarder"),
      m_sub(create_subscription<InMessage>(
          inputTopic, 10,
          std::bind(&DecodingForwarder::subscriptionCallback, this, _1))),
      m_pub(create_publisher<OutMessage>(outputTopic, 10)),
      m_decoder(decoderName) {}

DecodingForwarder::DecodingForwarder(const YAML::Node& config)
    : Node("ffmpeg_decoding_forwarder"),
      m_sub(create_subscription<InMessage>(
          config["input_topic"] ? config["input_topic"].as<std::string>()
                                : "/packet",
          10, std::bind(&DecodingForwarder::subscriptionCallback, this, _1))),
      m_pub(create_publisher<OutMessage>(
          config["output_topic"] ? config["output_topic"].as<std::string>()
                                 : "/image_decoded",
          10)),
      m_decoder(config["decoder"]["name"]
                    ? config["decoder"]["name"].as<std::string>().c_str()
                    : "hevc_cuvid"),
      m_options(config["decoder"]["options"]) {
  std::cout << m_options << '\n';
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
        RCLCPP_WARN(get_logger(), "Couldn't provide option %s",
                    option.first.as<std::string>().c_str());
    }

  m_decoder.setFrameWidth(msg.width);
  m_decoder.setFrameHeight(msg.height);
  m_decoder.setFramePixelFormat(static_cast<AVPixelFormat>(msg.pix_fmt));
  m_decoder.setBitrate(m_decoder.ACCORDINGLY);

  // * Assume 60 fps
  m_decoder.setFramerate(AVRational{60, 1});
  m_decoder.setTimeBase(AVRational{1, 60});

  // * Stolen from example. Not sure what it does but it is necessary for codecs
  //   to work
  m_decoder.setGOPSize(10);

  return m_decoder.open();
}

void DecodingForwarder::subscriptionCallback(const InMessage& msg) {
  int ret = 0;

  fillPacket(msg);

  if (m_packet.hasKeyframe()) m_recievedKeyframe = true;

  // * If not a single keyframe has been recieved, the decoder has nothing to
  //   start decoding from
  if (!m_recievedKeyframe) return;

  if (msg.width != m_decoder.frameWidth() ||
      msg.height != m_decoder.frameHeight() ||
      static_cast<AVPixelFormat>(msg.pix_fmt) != m_decoder.pixelFormat()) {
    ret = reconfigureDecoder(msg);
    if (ret < 0)
      throw std::runtime_error(
          "Could not reconfigure decoder after new message");
  }

  ret = m_decoder.decode(m_packet);
  if (ret < 0) throw std::runtime_error(avwrapper::getErrorMessage(ret));
  if (ret == 1) return;

  OutMessage outMsg;
  outMsg.header.set__frame_id(msg.header.frame_id);
  outMsg.header.set__stamp(get_clock()->now());
  outMsg.set__width(m_decoder.frameWidth());
  outMsg.set__height(m_decoder.frameHeight());
  outMsg.set__encoding("rgb8");
  outMsg.set__is_bigendian(AV_HAVE_BIGENDIAN);

  outMsg << m_decoder.getFrame();

  m_pub->publish(outMsg);

  // * Calculate latency
  if (msg.keyframe)
    RCLCPP_INFO(
        get_logger(), "Decoded keyframe with a delay of %lf ms",
        (get_clock()->now() - rclcpp::Time(outMsg.header.stamp)).seconds() *
            1000);
}

}  // namespace ffmpeg_bridge