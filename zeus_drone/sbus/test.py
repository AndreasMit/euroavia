from packet import PacketReader
import time




packet_reader = PacketReader('COM4', 100000)

counter = 0

while 1:
    # packet_reader.clear_display()
    # packet_reader.decode_sbus_display(in_hex=False)
    print(packet_reader.decode_sbus_channels())
    # time.sleep(0.5)