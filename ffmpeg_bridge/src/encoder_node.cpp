#include <yaml-cpp/yaml.h>

#include "ffmpeg_bridge/encoding_forwarder.hpp"

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: decoder_node <config_path>\n";
    return 0;
  }

  auto config = YAML::LoadFile(argv[1]);
  if (!config.IsMap()) {
    std::cout << "Could not load configuration file: " << argv[1] << '\n';
    return 1;
  }

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ffmpeg_bridge::EncodingForwarder>(config));

  rclcpp::shutdown();

  return 0;
}