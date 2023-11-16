from re import L
from typing import Callable
from rclpy.node import Node
from sensor_msgs.msg import Image
from ffmpeg_interfaces.msg import AVPacket
import libavpy as av
import utils


class BasicEncoder(Node):
    __slots__ = ["encoder", "options", "fps", "gop_size", "callback", "subscriber"]

    def __init__(
        self,
        name: str,
        config: dict,
        on_encode: Callable[[Image, AVPacket | None], None] | None = None,
    ):
        super().__init__(name)
        self.encoder = av.VideoEncoder(config["encoder"]["name"])

        self.options = config["encoder"].get("options", {})
        self.fps = av.Ratio(config["encoder"].get("fps", 30), 1)
        self.gop_size = config["encoder"].get("keyframe_every", 60)

        self.callback = on_encode
        self.subscriber = self.create_subscription(
            Image,
            config["input_topic"],
            (lambda msg: on_encode(msg, self.encode(msg)))
            if (on_encode is not None)
            else self.encode,
            10,
        )

    def reconfigure(self, msg: Image):
        self.encoder.reset()
        self.encoder.frame_width = msg.width
        self.encoder.frame_height = msg.height
        self.encoder.pixel_format = self.encoder.default_pixel_format()
        # This magic constant means that the bitrate will be calculated
        # according to given image dimensions and pixel format
        self.encoder.bitrate = -2

        self.encoder.fps = self.fps
        self.encoder.time_base = av.Ratio(1, 1) / self.fps
        self.encoder.gop_size = self.gop_size

        for key in self.options:
            self.encoder.set_option(key, self.options[key])

        self.encoder.open()

    def encode(self, msg: Image) -> AVPacket | None:
        if (
            msg.width != self.encoder.frame_width
            or msg.height != self.encoder.frame_height
        ):
            self.reconfigure(msg)
        self.frame = utils.from_image_msg(msg, True).to_format(
            self.encoder.pixel_format
        )

        ret = self.encoder.encode(self.frame)
        if ret == 1:
            return None
        if ret == 0:
            return self.encoder.packet
        else:
            raise RuntimeError("Failed to encode packet")


class BasicDecoder(Node):
    __slots__ = ["decoder", "options", "callback", "subscriber", "ready"]

    def __init__(
        self,
        name: str,
        config: dict,
        on_decode: Callable[[AVPacket, av.Frame | None], None] | None = None,
    ):
        super().__init__(name)
        self.decoder = av.VideoDecoder(config["decoder"]["name"])

        self.options = config["decoder"].get("options", {})
        self.callback = on_decode
        self.subscriber = self.create_subscription(
            AVPacket,
            config["input_topic"],
            (
                (lambda msg: on_decode(msg, self.decode(msg)))
                if on_decode is not None
                else self.decode
            ),
            10,
        )
        self.ready = False

    def reconfigure(self, msg: AVPacket):
        self.decoder.reset()
        self.decoder.frame_width = msg.width
        self.decoder.frame_height = msg.height
        self.decoder.pixel_format = av.PixelFormat(msg.coded_pix_fmt)

        # This magic constant means that the bitrate will be calculated
        # according to given image dimensions and pixel format
        self.decoder.bitrate = -2

        for key in self.options:
            self.decoder.set_option(key, self.options[key])

        self.decoder.open()

    def decode(self, msg: AVPacket) -> av.Frame | None:
        if msg.keyframe:
            self.ready = True
        if not self.ready:
            return None

        if (
            msg.width != self.decoder.frame_width
            or msg.height != self.decoder.frame_height
            or msg.coded_pix_fmt != self.decoder.pixel_format
        ):
            self.reconfigure(msg)

        self.packet = utils.from_packet_msg(msg)
        ret = self.decoder.decode(self.packet)

        if ret == 1:
            return None
        if ret == 0:
            return self.decoder.frame
        else:
            raise RuntimeError("Failed to decode packet")
