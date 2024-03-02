from packet import PacketReader
import time




packet_reader = PacketReader('COM4', 100000)

# 0f ac 64 a5 56 59 c9 4a  56 b2 92 95 ac 64 25 2b  |..d.VY.JV....d%+|
# 00000010  59 c9 4a 56 b2 92 95 00  00

while 1:
    # packet_reader.clear_display()
    # packet_reader.decode_sbus_display(in_hex=False)
    print(packet_reader.decode_sbus_channels())
    # time.sleep(0.5)