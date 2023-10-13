#include <ffmpeg_bridge/decoding_forwarder.hpp>
#include <yaml-cpp/yaml.h>

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout
        << "Usage: decoder_node <config file path>\n";
    return 0;
  }

  const char* ConfigPath = argv[1];
  auto config = YAML::LoadFile(ConfigPath);
  if (config.IsNull()) {
    std::cout << "Could not load configuration file: " << ConfigPath << '\n';
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ffmpeg_bridge::DecodingForwarder>(config));
  rclcpp::shutdown();

  return 0;
}