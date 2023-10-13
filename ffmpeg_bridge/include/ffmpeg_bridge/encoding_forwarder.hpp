#ifndef FFMPEG_BRIDGE__ENCODING_FORWARDER_H
#define FFMPEG_BRIDGE__ENCODING_FORWARDER_H

#include <rclcpp/rclcpp.hpp>
#include <av_wrapper/video_encoder.h>
#include <ffmpeg_interfaces/msg/av_packet.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

namespace ffmpeg_bridge {
/**
 * @brief A node that subscibes to and image topic and publishes encoded
 *        av_packets.
 */
class EncodingForwarder : public rclcpp::Node {
 public:  // * Types and friends
  typedef sensor_msgs::msg::Image InMessage;
  typedef ffmpeg_interfaces::msg::AVPacket OutMessage;

  enum { DefaultFPS = 30, DefaultGOPSize = 10 };

 public:
  EncodingForwarder(const std::string& inputTopic,
                    const std::string& outputTopic,
                    AVCodecID id = AV_CODEC_ID_HEVC, int fps = DefaultFPS);
  EncodingForwarder(const std::string inputTopic,
                    const std::string& outputTopic,
                    const std::string& encoderName = "libx265",
                    int fps = DefaultFPS);
  EncodingForwarder(const YAML::Node& config);

 private:
  int fillFrame(const InMessage& msg);
  int reconfigureEncoder(const InMessage& msg);

 private:
  void subscriptionCallback(const InMessage& msg);

 private:
  rclcpp::Subscription<InMessage>::SharedPtr m_sub;
  rclcpp::Publisher<OutMessage>::SharedPtr m_pub;
  avwrapper::VideoEncoder m_encoder;
  avwrapper::Frame m_frame;
  YAML::Node m_options;
  int m_fps = DefaultFPS;
  int m_gopsize = DefaultGOPSize;
  
};
}  // namespace ffmpeg_bridge

#endif  // FFMPEG_BRIDGE__ENCODING_FORWARDER_H