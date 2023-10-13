#include <av_wrapper/video_decoder.h>

#include <ffmpeg_interfaces/msg/av_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

namespace ffmpeg_bridge {
class DecodingForwarder : public rclcpp::Node {
 public:  // * Types and friends
  typedef ffmpeg_interfaces::msg::AVPacket InMessage;
  typedef sensor_msgs::msg::Image OutMessage;

 public:  // * Construction and assignment
  DecodingForwarder(const std::string& inputTopic,
                    const std::string& outputTopic,
                    AVCodecID codec = AV_CODEC_ID_HEVC);
  DecodingForwarder(const std::string& inputTopic,
                    const std::string& outputTopic,
                    const std::string& decoderName = "hevc");
  DecodingForwarder(const YAML::Node& config);

 private:
  int fillPacket(const InMessage& msg);
  int reconfigureDecoder(const InMessage& msg);
  void subscriptionCallback(const InMessage& msg);

 private:
  rclcpp::Subscription<InMessage>::SharedPtr m_sub;
  rclcpp::Publisher<OutMessage>::SharedPtr m_pub;
  avwrapper::VideoDecoder m_decoder;
  avwrapper::Packet m_packet;
  YAML::Node m_options;
  bool m_recievedKeyframe = false;
};
};  // namespace ffmpeg_bridge