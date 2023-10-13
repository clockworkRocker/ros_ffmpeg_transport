# Header of the original image
std_msgs/Header header

# Description of the uncompressed image
int16   pix_fmt
uint16  width
uint16  height

# Description of the compressed image
bool    keyframe

# Codec information
int16   codec_id
int64   pts

# Compressed data
uint32  size
uint8[] data