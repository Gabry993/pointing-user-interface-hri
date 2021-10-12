# the color orderining in data
uint8 RGB = 0
uint8 BGR = 1
# RGB or BGR
uint8 color_order 0

# the type of LEDs
uint8 APA102 = 0
uint8 WS2812 = 1
# APA102 or WS2812
uint8 type 0

# the channel id to which the strip is attached to, in 0..7
uint8 id

# uROS needs messages with unbounded size
# 3 byte (color) per pixel
uint8[] data
