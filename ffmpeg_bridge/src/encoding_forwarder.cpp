#include <ffmpeg_bridge/converters.h>
#include <ffmpeg_bridge/formats.h>

#include <ffmpeg_bridge/encoding_forwarder.hpp>

using std::placeholders::_1;
using Frame = avwrapper::Frame;
using Packet = avwrapper::Packet;

namespace ffmpeg_bridge {
EncodingForwarder::EncodingForwarder(const std::string& inputTopic,
                                     const std::string& outputTopic,
                                     AVCodecID id, int fps)
    : Node("ffmpeg_encoding_forwarder"),
      m_sub(create_subscription<InMessage>(
          inputTopic, 10,
          std::bind(&EncodingForwarder::subscriptionCallback, this, _1))),
      m_pub(create_publisher<OutMessage>(outputTopic, 10)),
      m_encoder(id),
      m_frame(),
      m_fps(fps) {}

EncodingForwarder::EncodingForwarder(const std::string inputTopic,
                                     const std::string& outputTopic,
                                     const std::string& encoderName, int fps)
    : Node("ffmpeg_encoding_forwarder"),
      m_sub(create_subscription<InMessage>(
          inputTopic, 10,
          std::bind(&EncodingForwarder::subscriptionCallback, this, _1))),
      m_pub(create_publisher<OutMessage>(outputTopic, 10)),
      m_encoder(encoderName),
      m_frame(),
      m_fps(fps) {}

EncodingForwarder::EncodingForwarder(const YAML::Node& config)
    : Node("ffmpeg_encoding_forwarder"),
      m_sub(create_subscription<InMessage>(
          config["input_topic"] ? config["input_topic"].as<std::string>()
                                : "/image_raw",
          10, std::bind(&EncodingForwarder::subscriptionCallback, this, _1))),
      m_pub(create_publisher<OutMessage>(
          config["output_topic"] ? config["output_topic"].as<std::string>()
                                 : "/packet",
          10)),
      m_encoder(config["encoder"]["name"]
                    ? config["encoder"]["name"].as<std::string>()
                    : "libx264"),
      m_options(config["encoder"]["options"]),
      m_fps(config["encoder"]["fps"]  //
                ? config["encoder"]["fps"].as<int>()
                : DefaultFPS),
      m_gopsize(config["encoder"]["keyframe_every"]
                    ? config["encoder"]["keyframe_every"].as<int>()
                    : DefaultGOPSize) {
  RCLCPP_INFO(get_logger(),
              "Created encoder. \nSubscribed to %s. Publishing packets on %s",
              m_sub->get_topic_name(), m_sub->get_topic_name());
}

void EncodingForwarder::subscriptionCallback(const InMessage& msg) {
  int ret;
  static int recievedImages = 0;
  static double latency = 0.;
  static double conversionTime = 0.;
  static double transferTime = 0.;

  auto tic = get_clock()->now();
  transferTime += (tic - msg.header.stamp).seconds() * 1000;
  fillFrame(msg);
  conversionTime += (get_clock()->now() - tic).seconds() * 1000;

  if (m_encoder.frameWidth() != m_frame.width() ||
      m_encoder.frameHeight() != m_frame.height() ||
      m_encoder.pixelFormat() != m_frame.format()) {
    ret = reconfigureEncoder(msg);
    if (ret < 0) throw std::runtime_error("Could not reconfigure the encoder");
  }
  ret = m_encoder.encode(m_frame);

  if (!ret) {
    auto compact = m_encoder.getPacket();

    OutMessage outMsg;

    outMsg.set__width(m_encoder.frameWidth());
    outMsg.set__height(m_encoder.frameHeight());
    outMsg.set__pix_fmt(m_encoder.pixelFormat());

    outMsg.set__keyframe(compact.hasKeyframe());
    outMsg.set__codec_id(compact.codecID());
    outMsg.set__pts(compact.pts());

    outMsg.set__size(compact.size());
    outMsg.data.resize(compact.size());
    outMsg.data.assign(compact.buffer(), compact.buffer() + compact.size());

    outMsg.header.set__frame_id(msg.header.frame_id);
    outMsg.header.set__stamp(get_clock()->now());

    m_pub->publish(outMsg);

    latency += (rclcpp::Time(outMsg.header.stamp) -
                rclcpp::Time(static_cast<uint32_t>(compact.pts() >> 32),
                             static_cast<int32_t>((compact.pts() << 32) >> 32),
                             RCL_ROS_TIME))
                   .seconds() *
               1000;

    ++recievedImages;

    if (recievedImages == 256) {
      RCLCPP_INFO(get_logger(),
                  "\nAvg. waiting for image time: %lf ms"
                  "\nAvg. conversion to frame time: %lf ms"
                  "\nAverage encoding time: %lf ms",
                  transferTime / recievedImages,
                  conversionTime / recievedImages, latency / recievedImages);
      latency = 0;
      conversionTime = 0;
      transferTime = 0.;
      recievedImages = 0;
    }
  };
}

int EncodingForwarder::reconfigureEncoder(const sensor_msgs::msg::Image& msg) {
  RCLCPP_INFO(this->get_logger(), "Configuring encoder...");
  int ret = 0;

  m_encoder.reset();

  m_encoder.setFrameWidth(msg.width + msg.width % 2);
  m_encoder.setFrameHeight(msg.height + msg.height % 2);

  m_encoder.setFramePixelFormat(m_encoder.defaultPixFmt());
  m_encoder.setBitrate(m_encoder.ACCORDINGLY);

  // * I have no idea what this number means. It just took it from an example
  m_encoder.setGOPSize(m_gopsize);
  m_encoder.setTimeBase(AVRational{1, m_fps});
  m_encoder.setFramerate(AVRational{m_fps, 1});

  if (m_options)
    for (auto option : m_options) {
      ret = m_encoder.setOption(option.first.as<std::string>(),
                                option.second.as<std::string>());
      if (ret < 0)
        RCLCPP_WARN(get_logger(), "Option not set: %s",
                    option.first.as<std::string>().c_str());
    }

  return m_encoder.open();
}

int EncodingForwarder::fillFrame(const InMessage& msg) {
  int correctW = msg.width + msg.width % 2;
  int correctH = msg.height + msg.height % 2;

  // * Reformat the frame if necessary
  if (msg.width != static_cast<unsigned>(m_frame.width()) ||
      msg.height != static_cast<unsigned>(m_frame.height()))
    m_frame = Frame(correctW, correctH, m_encoder.defaultPixFmt());

  // * Fill the frame
  m_frame.setPts(static_cast<int64_t>(msg.header.stamp.sec) << 32 |
                 static_cast<int64_t>(msg.header.stamp.nanosec));
  msg >> m_frame;

  return 0;
}

}  // namespace ffmpeg_bridge