from rclpy.node import Node
from sensor_msgs.msg import Image
from ffmpeg_interfaces.msg import AVPacket
import libavpy as av
import utils


class BasicEncoder(Node):
    def __init__(self, name: str, config: dict):
        super().__init__(name)
        self.encoder = av.VideoEncoder(config["encoder"]["name"])

        self.options = config["encoder"].get("options", {})
        self.fps = av.Ratio(config["encoder"].get("fps", 30), 1)
        self.gop_size = config["encoder"].get("keyframe_every", 60)

        self.subscriber = self.create_subscription(
            Image, config["input_topic"], self.encode, 10
        )

    def reconfigure(self, msg: Image):
        self.encoder.reset()
        self.encoder.frame_width = msg.width
        self.encoder.frame_height = msg.height
        self.encoder.pixel_format = utils.pixel_format(msg.encoding)
        # This magic constant means that the bitrate will be calculated
        # according to given image dimensions and pixel format
        self.encoder.bitrate = -2

        self.encoder.fps = self.fps
        self.encoder.time_base = av.Ratio(1, 1) / self.fps
        self.encoder.gop_size = self.gop_size

        for key, val in self.options:
            self.encoder.set_option(key, val)

        self.encoder.open()

    def encode(self, msg: Image):
        if (
            msg.width != self.encoder.frame_width
            or msg.height != self.encoder.frame_height
            or utils.pixel_format(msg.encoding) != self.encoder.pixel_format
        ):
            self.reconfigure(msg)

        ret = self.encoder.encode(utils.from_image(msg).to_format(self.encoder.pixel_format))
        if ret == 1:
            return None
        if ret == 0:
            return utils.messagify_packet(self.encoder.packet)
        else:
            raise RuntimeError("Failed to encode packet")
