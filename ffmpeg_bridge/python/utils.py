import sys

import numpy as np
import libavpy as av
from ffmpeg_interfaces.msg import AVPacket
from sensor_msgs.msg import Image


class hidden:
    SupportedPixelFormats = [
        av.PixelFormat.RGB24,
        av.PixelFormat.RGB48LE if sys.byteorder == "little" else av.PixelFormat.RGB48BE,
        av.PixelFormat.RGBA,
        av.PixelFormat.RGBA64LE
        if sys.byteorder == "little"
        else av.PixelFormat.RGBA64BE,
        av.PixelFormat.BGR24,
        av.PixelFormat.BGRA,
        av.PixelFormat.BGR48LE if sys.byteorder == "little" else av.PixelFormat.BGR48BE,
        av.PixelFormat.BGRA64LE
        if sys.byteorder == "little"
        else av.PixelFormat.BGRA64BE,
        av.PixelFormat.GRAY8,
        av.PixelFormat.GRAY16LE
        if sys.byteorder == "little"
        else av.PixelFormat.GRAY16BE,
        av.PixelFormat.UYVY422,
    ]

    SupportedROSEncodings = [
        "rgb8",
        "rgb16",
        "rgb16",
        "rgba8",
        "rgba16",
        "bgr8",
        "bgra8",
        "bgr16",
        "bgra16",
        "mono8",
        "mono16",
        "yuv422",
    ]


PixelFormat = dict(zip(hidden.SupportedROSEncodings, hidden.SupportedPixelFormats))
ROSEncoding = dict(zip(hidden.SupportedPixelFormats, hidden.SupportedROSEncodings))


def ros_encoding(format: av.PixelFormat) -> str:
    if format in ROSEncoding:
        return ROSEncoding[format]
    else:
        raise ValueError("Pixel format is not supported by ROS")


def pixel_format(encoding: str) -> av.PixelFormat:
    if encoding in hidden.SupportedROSEncodings:
        return PixelFormat[encoding]
    else:
        raise ValueError("Encoding is not currently supported by libav")


def messagify_frame(frame: av.Frame, copy: bool = True) -> Image:
    try:
        encoding = ros_encoding(av.PixelFormat(frame.format()))
    except:
        encoding = "rgb8"
    
    return Image(
        height=frame.height(),
        width=frame.width(),
        encoding=encoding,
        step=frame.row_step(),
        data=frame.to_numpy().tobytes(),
    )

def messagify_packet(packet: av.Packet, copy: bool = True, **kwargs) -> AVPacket:
    return AVPacket(
        pix_fmt=kwargs.pop("pix_fmt", av.PixelFormat.NONE),
        width=kwargs.pop("width", 0),
        height=kwargs.pop("height", 0),
        codec_id=kwargs.pop("codec_id", av.CodecID.NONE),
        coded_pix_fmt=kwargs.pop("coded_pix_fmt", av.CodecID.NONE),
        pts=packet.pts,
        keyframe=packet.has_keyframe(),
        data=packet.buffer()
    )


def from_image(img: Image, copy: bool = True) -> av.Frame:
    arr = np.array(img.data, dtype=np.uint8, copy=copy)
    return av.Frame.from_numpy(arr, pixel_format(img.encoding), copy)
